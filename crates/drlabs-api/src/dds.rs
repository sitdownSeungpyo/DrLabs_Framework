//! The thin layer over CycloneDDS. Every `unsafe` call into `cyclors` lives
//! here, behind four types: a participant, a topic, a writer of bytes and a
//! reader of bytes.
//!
//! Topics are *blob* topics: Cyclone carries the serialised sample as an
//! opaque buffer under a type name we choose, which is how a hand-written
//! CDR body (see [`crate::cdr`]) travels under the ROS 2 type name that
//! `rmw_cyclonedds` expects. The buffer handed to a writer, and the one a
//! reader hands back, both begin with the 4-byte encapsulation header.

use std::ffi::{c_void, CString};
use std::ptr;
use std::sync::Arc;

use cyclors::*;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DdsError(pub String);

impl std::fmt::Display for DdsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for DdsError {}

fn check(what: &str, rc: dds_return_t) -> Result<dds_entity_t, DdsError> {
    if rc < 0 {
        Err(DdsError(format!("{what}: dds error {rc}")))
    } else {
        Ok(rc)
    }
}

/// One domain participant. Deleting it deletes everything made from it, so
/// the server and the client each drop it last.
pub struct Participant {
    entity: dds_entity_t,
}

impl Participant {
    pub fn new(domain: u32) -> Result<Self, DdsError> {
        let entity = unsafe { dds_create_participant(domain, ptr::null(), ptr::null()) };
        check("create participant", entity).map(|entity| Self { entity })
    }

    pub fn entity(&self) -> dds_entity_t {
        self.entity
    }
}

impl Drop for Participant {
    fn drop(&mut self) {
        unsafe {
            dds_delete(self.entity);
        }
    }
}

/// A keyless blob topic under a given type name.
pub struct Topic {
    entity: dds_entity_t,
    // Cyclone keeps its own copies, but the C helper takes `*mut`; keeping
    // ours alive for the topic's life costs nothing and removes a question.
    _name: CString,
    _type_name: CString,
}

impl Topic {
    pub fn blob(participant: &Participant, name: &str, type_name: &str) -> Result<Self, DdsError> {
        let c_name = CString::new(name).map_err(|_| DdsError("topic name has a NUL".into()))?;
        let c_type = CString::new(type_name).map_err(|_| DdsError("type name has a NUL".into()))?;
        let entity = unsafe {
            cdds_create_blob_topic(
                participant.entity(),
                c_name.as_ptr() as *mut _,
                c_type.as_ptr() as *mut _,
                true,
            )
        };
        check(&format!("create topic {name}"), entity).map(|entity| Self {
            entity,
            _name: c_name,
            _type_name: c_type,
        })
    }

    pub fn entity(&self) -> dds_entity_t {
        self.entity
    }
}

/// The two QoS shapes this runtime uses.
pub struct Qos {
    ptr: *mut dds_qos_t,
}

impl Qos {
    /// Latest wins: a late command or state is worthless, so it is dropped
    /// rather than retried. `depth` samples are kept for a reader that is
    /// slower than its writer.
    pub fn best_effort(depth: i32) -> Self {
        unsafe {
            let ptr = dds_create_qos();
            dds_qset_reliability(ptr, dds_reliability_kind_DDS_RELIABILITY_BEST_EFFORT, 0);
            dds_qset_durability(ptr, dds_durability_kind_DDS_DURABILITY_VOLATILE);
            dds_qset_history(ptr, dds_history_kind_DDS_HISTORY_KEEP_LAST, depth);
            Self { ptr }
        }
    }

    /// Latched: the last sample is kept and handed to every reader that
    /// turns up later. For the description, which is written once.
    pub fn latched() -> Self {
        unsafe {
            let ptr = dds_create_qos();
            dds_qset_reliability(
                ptr,
                dds_reliability_kind_DDS_RELIABILITY_RELIABLE,
                i64::MAX, // DDS_INFINITY: never give up on a latched sample
            );
            dds_qset_durability(ptr, dds_durability_kind_DDS_DURABILITY_TRANSIENT_LOCAL);
            dds_qset_history(ptr, dds_history_kind_DDS_HISTORY_KEEP_LAST, 1);
            Self { ptr }
        }
    }

    fn as_ptr(&self) -> *const dds_qos_t {
        self.ptr
    }
}

impl Drop for Qos {
    fn drop(&mut self) {
        unsafe { dds_delete_qos(self.ptr) }
    }
}

/// Writes serialised samples (header included) to one topic.
pub struct Writer {
    entity: dds_entity_t,
    sertype: *const ddsi_sertype,
}

// Entity handles are plain integers and Cyclone is thread-safe; the sertype
// pointer is owned by Cyclone for the writer's life.
unsafe impl Send for Writer {}
unsafe impl Sync for Writer {}

impl Writer {
    pub fn new(participant: &Participant, topic: &Topic, qos: &Qos) -> Result<Self, DdsError> {
        let entity = unsafe {
            dds_create_writer(
                participant.entity(),
                topic.entity(),
                qos.as_ptr(),
                ptr::null(),
            )
        };
        let entity = check("create writer", entity)?;
        let mut sertype: *const ddsi_sertype = ptr::null();
        let rc = unsafe { dds_get_entity_sertype(entity, &mut sertype) };
        check("writer sertype", rc)?;
        if sertype.is_null() {
            return Err(DdsError("writer has no sertype".into()));
        }
        Ok(Self { entity, sertype })
    }

    /// Hands one serialised sample to Cyclone. `sample` starts with the
    /// encapsulation header.
    pub fn write(&self, sample: &[u8]) -> Result<(), DdsError> {
        let iov = ddsrt_iovec_t {
            iov_base: sample.as_ptr() as *mut c_void,
            iov_len: sample.len(),
        };
        unsafe {
            let serdata = ddsi_serdata_from_ser_iov(
                self.sertype,
                ddsi_serdata_kind_SDK_DATA,
                1,
                &iov,
                sample.len(),
            );
            if serdata.is_null() {
                return Err(DdsError("serdata from bytes failed".into()));
            }
            // dds_writecdr takes the reference, success or not.
            check("write", dds_writecdr(self.entity, serdata)).map(|_| ())
        }
    }
}

impl Drop for Writer {
    fn drop(&mut self) {
        unsafe {
            dds_delete(self.entity);
        }
    }
}

type Callback = Arc<dyn Fn(&[u8]) + Send + Sync>;

/// Reads serialised samples from one topic and hands each to a callback,
/// on Cyclone's receive thread. The callback has to be quick: it is on the
/// path every sample takes.
pub struct Reader {
    entity: dds_entity_t,
    listener: *mut dds_listener_t,
    // The listener's argument. Boxed so its address is stable; freed after
    // the reader, so no callback can outlive it.
    callback: *mut Callback,
}

unsafe impl Send for Reader {}
unsafe impl Sync for Reader {}

unsafe extern "C" fn on_data(reader: dds_entity_t, arg: *mut c_void) {
    let callback = &*(arg as *const Callback);
    loop {
        let mut serdata: *mut ddsi_serdata = ptr::null_mut();
        let mut info: dds_sample_info_t = std::mem::zeroed();
        let n = dds_takecdr(reader, &mut serdata, 1, &mut info, DDS_ANY_STATE);
        if n <= 0 || serdata.is_null() {
            break;
        }
        if info.valid_data {
            let size = ddsi_serdata_size(serdata) as usize;
            let mut iov: ddsrt_iovec_t = std::mem::zeroed();
            let reference = ddsi_serdata_to_ser_ref(serdata, 0, size, &mut iov);
            if !iov.iov_base.is_null() && iov.iov_len > 0 {
                let bytes = std::slice::from_raw_parts(iov.iov_base as *const u8, iov.iov_len);
                callback(bytes);
            }
            if !reference.is_null() {
                ddsi_serdata_to_ser_unref(reference, &iov);
            }
        }
        ddsi_serdata_unref(serdata);
    }
}

impl Reader {
    pub fn new(
        participant: &Participant,
        topic: &Topic,
        qos: &Qos,
        callback: Callback,
    ) -> Result<Self, DdsError> {
        let callback = Box::into_raw(Box::new(callback));
        let listener = unsafe { dds_create_listener(callback as *mut c_void) };
        unsafe { dds_lset_data_available(listener, Some(on_data)) };
        let entity = unsafe {
            dds_create_reader(participant.entity(), topic.entity(), qos.as_ptr(), listener)
        };
        match check("create reader", entity) {
            Ok(entity) => Ok(Self {
                entity,
                listener,
                callback,
            }),
            Err(error) => {
                unsafe {
                    dds_delete_listener(listener);
                    drop(Box::from_raw(callback));
                }
                Err(error)
            }
        }
    }
}

impl Drop for Reader {
    fn drop(&mut self) {
        unsafe {
            // Deleting the reader waits for a callback in flight; only then
            // is the argument safe to free.
            dds_delete(self.entity);
            dds_delete_listener(self.listener);
            drop(Box::from_raw(self.callback));
        }
    }
}
