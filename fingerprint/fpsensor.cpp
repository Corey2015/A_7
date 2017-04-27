#include <hardware/fingerprint.h>
#include "fpsensor.h"
#include "fpc_fpsensor.h"
#include "goodix_fpsensor.h"
#include <limits.h>
#include <unistd.h>
#include <sys/time.h>
#include <cutils/properties.h>
#include "fpsensor_l.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include <linux/msm_ion.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#ifdef TAC_TIME_MEASUREMENTS
#include <sys/time.h>
#endif

#include "fpsensor_km_tac_internal.h"
#include "fpsensor_km_tac.h"

#if defined(_FPC_IMAGE_STORING_) && defined(FPC_TAC_DEBUG)
#include "fpc_ict.h"
#endif /* _FPC_IMAGE_STORING_ && FPC_TAC_DEBUG */

#ifdef LOG_TAG
#undef LOG_TAG
#define LOG_TAG "FingerprintSensor"
#endif

#define ION_DEVICE       "/dev/ion"
#define ION_BUFFER_SIZE  (64*1024)

#define USECS_PER_SECOND (1000000)
#define MSECS_PER_SECOND (1000)

#if defined(_FPC_IMAGE_STORING_) && defined(FPC_TAC_DEBUG)
static fpc_lib_image_storage_t _fpsensor_image_storage;
static uint32_t _fpsensor_enroll_count = 0;

static uint32_t _fpsensor_store_verify_images(void *pHandle,
                                             fpc_lib_identify_data_t* data);

static uint32_t _fpsensor_store_enroll_images(void *pHandle,
                                            fpc_lib_enroll_data_t* data);

#endif /* _FPC_IMAGE_STORING_ && FPC_TAC_DEBUG */

int fpsensor_debug_retrieve_raw_image(void *pHandle,
                                 uint8_t* image,
                                 uint32_t size);

int fpsensor_debug_retrieve_enhanced_image(void *pHandle,
                                 uint8_t* image,
                                 uint32_t size);


int32_t fpsensor_alloc_ion(fpsensor_ion_info_t *ion_handle, uint32_t size)
{
  int32_t retval = 0;
  int32_t iret = 0;
  unsigned char *v_addr;
  struct ion_allocation_data ion_alloc_data;
  int32_t ion_fd;
  int32_t rc;
  struct ion_fd_data ifd_data;
  struct ion_handle_data handle_data;

  if (NULL == ion_handle)
  {
    LOGE("Invalid input parameter. ion_handle == NULL");
    return -1;
  }

  ion_fd  = open(ION_DEVICE, O_RDONLY);

  if (0 > ion_fd)
  {
    LOGE("Cannot open ION device");
    return -1;
  }

  ion_handle->ion_sbuffer = NULL;
  ion_handle->ifd_data_fd = 0;

  ion_alloc_data.len = (size + 4095) & (~4095);
  ion_alloc_data.align = 4096;
  ion_alloc_data.heap_id_mask = ION_HEAP(ION_QSECOM_HEAP_ID);
  ion_alloc_data.flags = 0;

  rc = ioctl(ion_fd, ION_IOC_ALLOC, &ion_alloc_data);
  if (0 != rc)
  {
    LOGE("ioctl (ION_IOC_ALLOC) failed, return value: %d", rc);
    goto alloc_fail;
  }

  if (ion_alloc_data.handle)
  {
    ifd_data.handle = ion_alloc_data.handle;
  }
  else
  {
    LOGE("Cannot allocate ION data, data handle is NULL");
    goto alloc_fail;
  }

  rc = ioctl(ion_fd, ION_IOC_MAP, &ifd_data);

  if (0 != rc)
  {
    LOGE("ioctl (ION_IOC_MAP) failed, return value: %d", rc);
    goto ioctl_fail;
  }

  v_addr = (unsigned char *) mmap(NULL, ion_alloc_data.len,
                                  PROT_READ | PROT_WRITE,
                                  MAP_SHARED, ifd_data.fd, 0);
  if (MAP_FAILED == v_addr)
  {
    LOGE("mmap failed");
    retval = -1;
    goto map_fail;
  }

  ion_handle->ion_fd = ion_fd;
  ion_handle->ifd_data_fd = ifd_data.fd;
  ion_handle->ion_sbuffer = v_addr;
  ion_handle->ion_alloc_handle.handle = ion_alloc_data.handle;
  ion_handle->sbuf_len = size;

  return retval;

map_fail:
  if (NULL != ion_handle->ion_sbuffer)
  {
    retval = munmap(ion_handle->ion_sbuffer, ion_alloc_data.len);
    if (retval)
      LOGE("munmap failed with retval = %d\n", retval);
  }

ioctl_fail:
  handle_data.handle = ion_alloc_data.handle;

  if (ion_handle->ifd_data_fd)
    close(ion_handle->ifd_data_fd);

  iret = ioctl(ion_fd, ION_IOC_FREE, &handle_data);

  if (iret)
    LOGE("ioctl (ION_IOC_FREE) failed, return value: %d", iret);

alloc_fail:
  if (ion_fd)
    close(ion_fd);

  return retval;
}

int32_t fpsensor_dealloc_ion(fpsensor_ion_info_t *ion_handle)
{
  int32_t retval = 0;
  struct ion_handle_data handle_data;

  retval = munmap(ion_handle->ion_sbuffer, (ion_handle->sbuf_len + 4095) & (~4095));
  if (retval)
    LOGE("munmap failed with retval = %d\n", retval);

  handle_data.handle = ion_handle->ion_alloc_handle.handle;

  close(ion_handle->ifd_data_fd);

  retval = ioctl(ion_handle->ion_fd, ION_IOC_FREE, &handle_data);
  if (retval)
    LOGE("ioctl (ION_IOC_FREE) failed, return value: %d", retval);

  close(ion_handle->ion_fd);
  return retval;
}

static int fpsensor_send_encapsulated_key(const fpsensor_handle_internal_t* tac_handle,
                                         const uint8_t* encapsulated_key,
                                         uint32_t size_encapsulated_key)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_debug_image_cmd_t *send_cmd = NULL;
  fpsensor_generic_response_t *resp = NULL;
  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;

  struct QSEECom_ion_fd_info ion_fd_info;
  fpsensor_ion_info_t ion_handle;

  LOGD("%s begin", __func__);

  ion_handle.ion_fd = 0;
  ion_handle.ion_alloc_handle.handle = 0;

  retval = fpsensor_alloc_ion(&ion_handle, size_encapsulated_key);
  if (0 != retval)
  {
    LOGE("fpsensor_alloc_ion failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_MEMORY;
  }

  memset(&ion_fd_info, 0, sizeof(struct QSEECom_ion_fd_info));

  ion_fd_info.data[0].fd = ion_handle.ifd_data_fd;
  ion_fd_info.data[0].cmd_buf_offset = sizeof(send_cmd->cmd_id);

  
  send_cmd = (fpsensor_debug_image_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;

  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_debug_image_cmd_t));
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

  resp = (fpsensor_generic_response_t *) (tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len);

  send_cmd->cmd_id = FINGERPRINT_TEE_SET_ENCAPSULATED_KEY_CMD_ID;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  send_cmd->image_data = (uint32_t)(uintptr_t) ion_handle.ion_sbuffer;
  send_cmd->image_len = size_encapsulated_key;

  if (encapsulated_key != NULL) {
    memcpy(ion_handle.ion_sbuffer, encapsulated_key, size_encapsulated_key);
  }

  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;

  retval = QSEECom_send_modified_cmd(tac_handle->pQSEEComHandle,
                                     send_cmd,
                                     send_cmd_len,
                                     resp,
                                     resp_len,
                                     &ion_fd_info);

  
  if (0 != retval)
  {
    LOGE("QSEECom_send_modified_cmd failed with retval = %d\n", retval);
    fpsensor_dealloc_ion(&ion_handle);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  fpsensor_dealloc_ion(&ion_handle);

  LOGD("%s end", __func__);

  return resp->status;
}

static int fpsensor_send_cmd_rc(const fpsensor_handle_internal_t* tac_handle,
                            const fpsensor_ta_cmd_t cmd,
                            uint32_t data,
							uint32_t* result)
{
  int retval = FPSENSOR_TAC_OK;

  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;
  fpsensor_generic_cmd_t *send_cmd = NULL;
  fpsensor_generic_response_t *resp = NULL;

  LOGD("%s begin", __func__);
  
  send_cmd = (fpsensor_generic_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;
  send_cmd->cmd_id = cmd;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  send_cmd->data = data;

  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_cmd_t));
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

  resp = (fpsensor_generic_response_t *) tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len;
  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;

  retval = QSEECom_send_cmd(tac_handle->pQSEEComHandle,
                            send_cmd,
                            send_cmd_len,
                            resp,
                            resp_len);
  
  if (0 != retval)
  {
    LOGE("QSEECom_send_cmd failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }
  if(result)
	  *result = resp->data;
  LOGD("%s end", __func__);

  return resp->status;
}
;
int fpsensor_send_cmd(const fpsensor_handle_internal_t* tac_handle,
                            const fpsensor_ta_cmd_t cmd,
                            uint32_t data)
{
  int retval = FPSENSOR_TAC_OK;

  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;
  fpsensor_generic_cmd_t *send_cmd = NULL;
  fpsensor_generic_response_t *resp = NULL;

  LOGD("%s begin", __func__);
  
  send_cmd = (fpsensor_generic_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;
  send_cmd->cmd_id = cmd;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  send_cmd->data = data;

  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_cmd_t));
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

  resp = (fpsensor_generic_response_t *) tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len;
  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;

  retval = QSEECom_send_cmd(tac_handle->pQSEEComHandle,
                            send_cmd,
                            send_cmd_len,
                            resp,
                            resp_len);
  
  if (0 != retval)
  {
    LOGE("QSEECom_send_cmd failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  LOGD("%s end", __func__);

  return resp->status;
}

static int fpsensor_send_cmd64(const fpsensor_handle_internal_t* tac_handle,
                            const fpsensor_ta_cmd_t cmd,
                            uint64_t data)
{
  int retval = FPSENSOR_TAC_OK;

  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;
  fpsensor_cmd64_t *send_cmd = NULL;
  fpsensor_generic_response_t *resp = NULL;

  LOGD("%s begin", __func__);
  
  send_cmd = (fpsensor_cmd64_t *) tac_handle->pQSEEComHandle->ion_sbuffer;
  send_cmd->cmd_id = cmd;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  send_cmd->data = data;

  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_cmd64_t));
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

  resp = (fpsensor_generic_response_t *) tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len;
  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;

  retval = QSEECom_send_cmd(tac_handle->pQSEEComHandle,
                            send_cmd,
                            send_cmd_len,
                            resp,
                            resp_len);
  
  if (0 != retval)
  {
    LOGE("QSEECom_send_cmd failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  LOGD("%s end", __func__);

  return resp->status;
}

static int fpsensor_send_set_arbitrary_data_cmd(
    const fpsensor_handle_internal_t* tac_handle,
    const fpsensor_ta_cmd_t cmd,
    uint8_t* data,
    uint32_t data_size)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_debug_image_cmd_t *send_cmd = NULL;
  fpsensor_generic_response_t *resp = NULL;
  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;

  struct QSEECom_ion_fd_info ion_fd_info;
  fpsensor_ion_info_t ion_handle;

  LOGD("%s begin", __func__);

  ion_handle.ion_fd = 0;
  ion_handle.ion_alloc_handle.handle = 0;

  retval = fpsensor_alloc_ion(&ion_handle, data_size);
  if (0 != retval)
  {
    LOGE("fpsensor_alloc_ion failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_MEMORY;
  }

  memset(&ion_fd_info, 0, sizeof(struct QSEECom_ion_fd_info));

  ion_fd_info.data[0].fd = ion_handle.ifd_data_fd;
  ion_fd_info.data[0].cmd_buf_offset = sizeof(send_cmd->cmd_id);

  
  send_cmd = (fpsensor_debug_image_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;

  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_debug_image_cmd_t));
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

  resp = (fpsensor_generic_response_t *) (tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len);

  send_cmd->cmd_id = cmd;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  send_cmd->image_data = (uint32_t)(uintptr_t) ion_handle.ion_sbuffer;
  send_cmd->image_len = data_size;
  fpsensor_memcpy((unsigned char *)ion_handle.ion_sbuffer, data, data_size);

  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;

  retval = QSEECom_send_modified_cmd(tac_handle->pQSEEComHandle,
                                     send_cmd,
                                     send_cmd_len,
                                     resp,
                                     resp_len,
                                     &ion_fd_info);

  
  if (0 != retval)
  {
    LOGE("QSEECom_send_modified_cmd failed with retval = %d\n", retval);
    fpsensor_dealloc_ion(&ion_handle);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  fpsensor_dealloc_ion(&ion_handle);

  LOGD("%s end", __func__);

  return resp->status;
}


int fpsensor_send_get_arbitrary_data_cmd(
    const fpsensor_handle_internal_t* tac_handle,
    const fpsensor_ta_cmd_t cmd,
    uint8_t* data,
    uint32_t data_size)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_debug_image_cmd_t *send_cmd = NULL;
  fpsensor_generic_response_t *resp = NULL;
  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;

  struct QSEECom_ion_fd_info ion_fd_info;
  fpsensor_ion_info_t ion_handle;

  LOGD("%s begin", __func__);

  ion_handle.ion_fd = 0;
  ion_handle.ion_alloc_handle.handle = 0;

  retval = fpsensor_alloc_ion(&ion_handle, data_size);
  if (0 != retval)
  {
    LOGE("fpsensor_alloc_ion failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_MEMORY;
  }

  memset(&ion_fd_info, 0, sizeof(struct QSEECom_ion_fd_info));

  ion_fd_info.data[0].fd = ion_handle.ifd_data_fd;
  ion_fd_info.data[0].cmd_buf_offset = sizeof(send_cmd->cmd_id);

  
  send_cmd = (fpsensor_debug_image_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;
  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_debug_image_cmd_t));
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

  resp = (fpsensor_generic_response_t *) (tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len);

  send_cmd->cmd_id = cmd;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  send_cmd->image_data = (uint32_t)(uintptr_t) ion_handle.ion_sbuffer;
  send_cmd->image_len = data_size;

  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;

  retval = QSEECom_send_modified_cmd(tac_handle->pQSEEComHandle,
                                     send_cmd,
                                     send_cmd_len,
                                     resp,
                                     resp_len,
                                     &ion_fd_info);

  
  if (0 != retval)
  {
    LOGE("QSEECom_send_modified_cmd failed with retval = %d\n", retval);
    fpsensor_dealloc_ion(&ion_handle);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  fpsensor_memcpy(data, (unsigned char *)ion_handle.ion_sbuffer, data_size);

  fpsensor_dealloc_ion(&ion_handle);

  LOGD("%s end", __func__);

  return resp->status;
}

static int fpsensor_send_read_template_cmd(
    const fpsensor_handle_internal_t* tac_handle,
    uint32_t data_size,
    fpsensor_db_type_t db_type)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_debug_image_cmd_t *send_cmd = NULL;
  fpsensor_generic_response_t *resp = NULL;
  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;

  struct QSEECom_ion_fd_info ion_fd_info;
  fpsensor_ion_info_t ion_handle;

  LOGD("%s begin", __func__);

  ion_handle.ion_fd = 0;
  ion_handle.ion_alloc_handle.handle = 0;

  retval = fpsensor_alloc_ion(&ion_handle, data_size);
  if (0 != retval)
  {
    LOGE("fpsensor_alloc_ion failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_MEMORY;
  }

  memset(&ion_fd_info, 0, sizeof(struct QSEECom_ion_fd_info));

  ion_fd_info.data[0].fd = ion_handle.ifd_data_fd;
  ion_fd_info.data[0].cmd_buf_offset = sizeof(send_cmd->cmd_id);

  
  send_cmd = (fpsensor_debug_image_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;
  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_debug_image_cmd_t));
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

  resp = (fpsensor_generic_response_t *) (tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len);

  send_cmd->cmd_id = FINGERPRINT_TEE_GET_TEMPLATE_DB_CMD_ID;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  send_cmd->image_data = (uint32_t)(uintptr_t) ion_handle.ion_sbuffer;
  send_cmd->image_len = data_size;
  send_cmd->misc_data = db_type;

  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;

  retval = QSEECom_send_modified_cmd(tac_handle->pQSEEComHandle,
                                     send_cmd,
                                     send_cmd_len,
                                     resp,
                                     resp_len,
                                     &ion_fd_info);

  
  if (0 != retval)
  {
    LOGE("QSEECom_send_modified_cmd failed with retval = %d\n", retval);
    fpsensor_dealloc_ion(&ion_handle);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  retval = fpsensor_write_templates_in_hlos_fs(tac_handle,
          (unsigned char *)ion_handle.ion_sbuffer,
          data_size,
          db_type);
  if (FPSENSOR_TAC_OK != retval)
  {
    LOGE("saving template db failed with retval = %d\n", retval);
    fpsensor_dealloc_ion(&ion_handle);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  fpsensor_dealloc_ion(&ion_handle);

  LOGD("%s end", __func__);

  return resp->status;
}

static int fpsensor_send_write_template_cmd(
    const fpsensor_handle_internal_t* tac_handle,
    uint32_t data_size,
    fpsensor_db_type_t db_type)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_debug_image_cmd_t *send_cmd = NULL;
  fpsensor_generic_response_t *resp = NULL;
  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;

  struct QSEECom_ion_fd_info ion_fd_info;
  fpsensor_ion_info_t ion_handle;

  LOGD("%s begin", __func__);

  ion_handle.ion_fd = 0;
  ion_handle.ion_alloc_handle.handle = 0;
  uint32_t alloc_size = data_size;

  // Dummy allocation size
  if (data_size == 0)
  {
    alloc_size = 4096;
  }

  retval = fpsensor_alloc_ion(&ion_handle, alloc_size);
  if (0 != retval)
  {
    LOGE("fpsensor_alloc_ion failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_MEMORY;
  }

  memset(&ion_fd_info, 0, sizeof(struct QSEECom_ion_fd_info));

  ion_fd_info.data[0].fd = ion_handle.ifd_data_fd;
  ion_fd_info.data[0].cmd_buf_offset = sizeof(send_cmd->cmd_id);

  
  send_cmd = (fpsensor_debug_image_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;

  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_debug_image_cmd_t));
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

  resp = (fpsensor_generic_response_t *) (tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len);

  send_cmd->cmd_id = FINGERPRINT_TEE_SET_TEMPLATE_DB_CMD_ID;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  send_cmd->image_data = (uint32_t)(uintptr_t) ion_handle.ion_sbuffer;
  send_cmd->image_len = data_size;
  send_cmd->misc_data = db_type;

  // No file exists, if data_size = 0;
  if (data_size > 0)
  {
    retval = fpsensor_read_templates_from_hlos_fs(tac_handle,
                                                 (unsigned char *)ion_handle.ion_sbuffer,
                                                 data_size,
                                                 db_type);
    if (FPSENSOR_TAC_OK != retval)
    {
      LOGE("reading template db failed with retval = %d\n", retval);
      
      fpsensor_dealloc_ion(&ion_handle);
      return FPSENSOR_TAC_ERROR_GENERAL;
    }
  }

  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;

  retval = QSEECom_send_modified_cmd(tac_handle->pQSEEComHandle,
                                     send_cmd,
                                     send_cmd_len,
                                     resp,
                                     resp_len,
                                     &ion_fd_info);

  
  if (0 != retval)
  {
    LOGE("QSEECom_send_modified_cmd failed with retval = %d\n", retval);
    fpsensor_dealloc_ion(&ion_handle);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  fpsensor_dealloc_ion(&ion_handle);

  LOGD("%s end", __func__);

  return resp->status;
}

static int fpsensor_send_get_indices_cmd(
    const fpsensor_handle_internal_t* tac_handle,
    uint32_t* indices, uint32_t* index_count)
{
  int retval = FPSENSOR_TAC_OK;
  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;
  fpsensor_generic_cmd_t *send_cmd = NULL;
  fpsensor_get_indices_response_t *resp = NULL;

  LOGD("%s begin", __func__);

  if (NULL == tac_handle)
  {
    LOGE("Invalid input parameter. tac_handle == NULL");
    return FPSENSOR_TAC_ERROR_PARAMETER;
  }

  if (NULL == indices)
  {
    LOGE("Invalid input parameter. indices == NULL");
    return FPSENSOR_TAC_ERROR_PARAMETER;
  }

  
  send_cmd = (fpsensor_generic_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;
  send_cmd->cmd_id = FINGERPRINT_TEE_GET_INDICES_CMD_ID;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  send_cmd->len = *index_count;
  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_cmd_t));

  resp = (fpsensor_get_indices_response_t *) tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len;
  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_get_indices_response_t));

  retval = QSEECom_send_cmd(tac_handle->pQSEEComHandle,
                                     send_cmd,
                                     send_cmd_len,
                                     resp,
                                     resp_len);

  
  if (0 != retval)
  {
    LOGE("QSEECom_send_cmd failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  if (resp->indices_len > *index_count)
  {
    LOGE("Number of indices in the database larger than expected: %d\n", resp->indices_len);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  LOGD("Number of indices in the database: %d\n", resp->indices_len);

  *index_count = resp->indices_len;
  fpsensor_memcpy(indices, resp->indices, resp->indices_len * sizeof(uint32_t));

  LOGD("%s end", __func__);

  return resp->status;
}

static int fpsensor_send_get_template_id_from_index_cmd(const fpsensor_handle_internal_t* tac_handle,
                                       uint32_t index,
                                       uint32_t* id)
{
  int retval = FPSENSOR_TAC_OK;
  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;
  fpsensor_generic_cmd_t *send_cmd = NULL;
  fpsensor_generic_response_t *resp = NULL;

  LOGD("%s begin", __func__);

  if (NULL == tac_handle)
  {
    LOGE("Invalid input parameter. tac_handle == NULL");
    return FPSENSOR_TAC_ERROR_PARAMETER;
  }

  if (NULL == id)
  {
    LOGE("Invalid input parameter. data == NULL");
    return FPSENSOR_TAC_ERROR_PARAMETER;
  }

  
  send_cmd = (fpsensor_generic_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;
  send_cmd->cmd_id = FINGERPRINT_TEE_GET_TEMPLATE_ID_FROM_INDEX_CMD_ID;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  
  send_cmd->data = index;
  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_cmd_t));

  resp = (fpsensor_generic_response_t *) tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len;
  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

  retval = QSEECom_send_cmd(tac_handle->pQSEEComHandle,
                                    send_cmd,
                            send_cmd_len,
                            resp,
                            resp_len);

  
  if (0 != retval)
  {
    LOGE("QSEECom_send_cmd failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  *id = send_cmd->data;

  LOGD("%s end", __func__);

  return resp->status;
}

int fpsensor_send_get_uint32_cmd(const fpsensor_handle_internal_t* tac_handle,
                                       const fpsensor_ta_cmd_t cmd,
                                       uint32_t* data)
{
  int retval = FPSENSOR_TAC_OK;
  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;
  fpsensor_generic_get_uint32_cmd_t *send_cmd = NULL;
  fpsensor_generic_response_t *resp = NULL;

  LOGD("%s begin", __func__);

  if (NULL == tac_handle)
  {
    LOGE("Invalid input parameter. tac_handle == NULL");
    return FPSENSOR_TAC_ERROR_PARAMETER;
  }

  if (NULL == data)
  {
    LOGE("Invalid input parameter. data == NULL");
    return FPSENSOR_TAC_ERROR_PARAMETER;
  }

  
  send_cmd = (fpsensor_generic_get_uint32_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;
  send_cmd->cmd_id = cmd;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  
  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_get_uint32_cmd_t));

  resp = (fpsensor_generic_response_t *) tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len;
  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

  retval = QSEECom_send_cmd(tac_handle->pQSEEComHandle,
                            send_cmd,
                            send_cmd_len,
                            resp,
                            resp_len);

  
  if (0 != retval)
  {
    LOGE("QSEECom_send_cmd failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  *data = send_cmd->data;

  LOGD("%s end", __func__);

  return resp->status;
}

static int fpsensor_send_get_uint64_cmd(const fpsensor_handle_internal_t* tac_handle,
                                       const fpsensor_ta_cmd_t cmd,
                                       uint64_t* data)
{
  int retval = FPSENSOR_TAC_OK;
  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;
  fpsensor_generic_get_uint64_cmd_t *send_cmd = NULL;
  fpsensor_generic_response_t *resp = NULL;

  LOGD("%s begin", __func__);

  if (NULL == tac_handle)
  {
    LOGE("Invalid input parameter. tac_handle == NULL");
    return FPSENSOR_TAC_ERROR_PARAMETER;
  }

  if (NULL == data)
  {
    LOGE("Invalid input parameter. data == NULL");
    return FPSENSOR_TAC_ERROR_PARAMETER;
  }

  
  send_cmd = (fpsensor_generic_get_uint64_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;
  send_cmd->cmd_id = cmd;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  
  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_get_uint64_cmd_t));

  resp = (fpsensor_generic_response_t *) tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len;
  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

  retval = QSEECom_send_cmd(tac_handle->pQSEEComHandle,
                            send_cmd,
                            send_cmd_len,
                            resp,
                            resp_len);

  
  if (0 != retval)
  {
    LOGE("QSEECom_send_cmd failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  *data = send_cmd->data;

  LOGD("%s end", __func__);

  return resp->status;
}

static int fpsensor_send_set_uint64_cmd(const fpsensor_handle_internal_t* tac_handle,
                                       const fpsensor_ta_cmd_t cmd,
                                       uint64_t data)
{
  int retval = FPSENSOR_TAC_OK;
  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;
  fpsensor_generic_get_uint64_cmd_t *send_cmd = NULL;
  fpsensor_generic_response_t *resp = NULL;

  LOGD("%s begin", __func__);

  if (NULL == tac_handle)
  {
    LOGE("Invalid input parameter. tac_handle == NULL");
    return FPSENSOR_TAC_ERROR_PARAMETER;
  }

  
  send_cmd = (fpsensor_generic_get_uint64_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;
  send_cmd->cmd_id = cmd;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  
  send_cmd->data = data;
  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_get_uint64_cmd_t));

  resp = (fpsensor_generic_response_t *) tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len;
  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

  retval = QSEECom_send_cmd(tac_handle->pQSEEComHandle,
                            send_cmd,
                            send_cmd_len,
                            resp,
                            resp_len);
  
  if (0 != retval)
  {
    LOGE("QSEECom_send_cmd failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  LOGD("%s end", __func__);

  return resp->status;
}

static int fpsensor_send_get_template_size_cmd(const fpsensor_handle_internal_t* tac_handle,
                                              fpsensor_db_type_t db_type,
                                              uint32_t* size)
{
  int retval = FPSENSOR_TAC_OK;
  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;
  fpsensor_get_template_size_cmd_t *send_cmd = NULL;
  fpsensor_generic_response_t *resp = NULL;

  LOGD("%s begin", __func__);

  if (NULL == tac_handle)
  {
    LOGE("Invalid input parameter. tac_handle == NULL");
    return FPSENSOR_TAC_ERROR_PARAMETER;
  }

  if (NULL == size)
  {
    LOGE("Invalid input parameter. size == NULL");
    return FPSENSOR_TAC_ERROR_PARAMETER;
  }
  
  send_cmd = (fpsensor_get_template_size_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;
  send_cmd->cmd_id = FINGERPRINT_TEE_GET_TEMPLATE_DB_SIZE_CMD_ID;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  
  send_cmd->db_type = db_type;
  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_get_template_size_cmd_t));

  resp = (fpsensor_generic_response_t *) tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len;
  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

  retval = QSEECom_send_cmd(tac_handle->pQSEEComHandle,
                            send_cmd,
                            send_cmd_len,
                            resp,
                            resp_len);
  
  if (0 != retval)
  {
    LOGE("QSEECom_send_cmd failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  *size = send_cmd->size;

  LOGD("%s end", __func__);

  return resp->status;
}

static int fpsensor_send_get_auth_data_cmd(
    const fpsensor_handle_internal_t* tac_handle,
    const uint8_t* nonce,
    const char* secAppName,
    fpsensor_auth_data_t* pAuthData)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_get_auth_data_request_t *req = NULL;
  fpsensor_get_auth_data_response_t *resp = NULL;
  int32_t req_len = 0;
  int32_t resp_len = 0;

  struct QSEECom_ion_fd_info ion_fd_info;
  fpsensor_ion_info_t ion_handle;

  LOGD("%s begin", __func__);

  ion_handle.ion_fd = 0;
  ion_handle.ion_alloc_handle.handle = 0;

  retval = fpsensor_alloc_ion(&ion_handle, QC_USER_VERIFICATION_TOKEN_LEN);
  if (0 != retval)
  {
    LOGE("fpsensor_alloc_ion failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_MEMORY;
  }

  memset(&ion_fd_info, 0, sizeof(struct QSEECom_ion_fd_info));

  ion_fd_info.data[0].fd = ion_handle.ifd_data_fd;
  ion_fd_info.data[0].cmd_buf_offset = sizeof(req->cmd_id);

  req_len = QSEECOM_ALIGN(sizeof(fpsensor_get_auth_data_request_t));
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_get_auth_data_response_t));

  
  req = (fpsensor_get_auth_data_request_t *) tac_handle->pQSEEComHandle->ion_sbuffer;
  resp = (fpsensor_get_auth_data_response_t *) (tac_handle->pQSEEComHandle->ion_sbuffer + req_len);

  req->cmd_id = FINGERPRINT_TEE_GET_AUTH_DATA_CMD_ID;
  req->cmd_id ^= tac_handle->serialnum;
  memcpy(req->nonce, nonce, QC_AUTH_NONCE);
  memcpy(req->secAppName, secAppName, QC_AUTH_SEC_APP_NAME_LEN);
  req->encapsulatedResultIon = (uint32_t)(uintptr_t) ion_handle.ion_sbuffer;

  retval = QSEECom_send_modified_cmd(tac_handle->pQSEEComHandle,
                                     req,
                                     req_len,
                                     resp,
                                     resp_len,
                                     &ion_fd_info);

  
  if (0 != retval)
  {
    LOGE("QSEECom_send_modified_cmd failed with retval = %d\n", retval);
    fpsensor_dealloc_ion(&ion_handle);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }


  pAuthData->result = (qc_auth_code_t)resp->result;
  pAuthData->userId = resp->userId;
  pAuthData->userEntityId = resp->userEntifyId;
  pAuthData->encapsulatedResultLength = resp->encapsulatedResultLength;
  memcpy(pAuthData->encapsulatedResult, ion_handle.ion_sbuffer, QC_USER_VERIFICATION_TOKEN_LEN);

  fpsensor_dealloc_ion(&ion_handle);

  LOGD("%s end", __func__);

  return retval;
}

static int fpsensor_send_template_db(void *pHandle, fpsensor_db_type_t db_type)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;
  uint32_t size = 0;

  LOGD("%s begin", __func__);

  retval = fpc_get_template_size(tac_handle, &size, db_type);
  if (FPSENSOR_TAC_OK != retval)
  {
    LOGD("failed to get template size retval: %d; continuing", retval);
    size = 0; // If template doesn't exist, signal to TA to create empty DB
  }

  retval = fpsensor_send_write_template_cmd(tac_handle, size, db_type);
  if (FPSENSOR_TAC_OK != retval)
  {
    LOGE("tac_fpc_send_write_template failed with retval: %d", retval);
  }

  LOGD("%s end", __func__);

  return retval;
}

int fpsensor_send_set_template_db_path(const fpsensor_handle_internal_t* tac_handle,
                                      const char* path,
                                      uint32_t len,
                                      fpsensor_db_type_t db_type)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_set_template_db_path_cmd_t *send_cmd = NULL;
  fpsensor_generic_response_t *resp = NULL;
  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;

  struct QSEECom_ion_fd_info ion_fd_info;
  fpsensor_ion_info_t ion_handle;

  LOGD("%s begin", __func__);

  ion_handle.ion_fd = 0;
  ion_handle.ion_alloc_handle.handle = 0;

  retval = fpsensor_alloc_ion(&ion_handle, len);
  if (0 != retval)
  {
    LOGE("fpsensor_alloc_ion failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_MEMORY;
  }

  memset(&ion_fd_info, 0, sizeof(struct QSEECom_ion_fd_info));

  ion_fd_info.data[0].fd = ion_handle.ifd_data_fd;
  ion_fd_info.data[0].cmd_buf_offset = sizeof(send_cmd->cmd_id);
  
  send_cmd = (fpsensor_set_template_db_path_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;

  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_set_template_db_path_cmd_t));
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

  resp = (fpsensor_generic_response_t *) (tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len);

  send_cmd->cmd_id = FINGERPRINT_TEE_SET_TEMPLATE_DB_PATH_CMD_ID;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  
  send_cmd->path = (uint32_t)(uintptr_t) ion_handle.ion_sbuffer;
  send_cmd->path_length = len;
  send_cmd->db_type = db_type;
  fpsensor_memcpy((unsigned char *)ion_handle.ion_sbuffer, path, len);

  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;

  retval = QSEECom_send_modified_cmd(tac_handle->pQSEEComHandle,
                                     send_cmd,
                                     send_cmd_len,
                                     resp,
                                     resp_len,
                                     &ion_fd_info);
  
  if (0 != retval)
  {
    LOGE("QSEECom_send_modified_cmd failed with retval = %d\n", retval);
    fpsensor_dealloc_ion(&ion_handle);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  fpsensor_dealloc_ion(&ion_handle);

  LOGD("%s end", __func__);

  return resp->status;
}

int fpsensor_send_delete_fingerprint_set(const fpsensor_handle_internal_t* tac_handle, int32_t fingerprint_set_key,
        const char* path, uint32_t len)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_delete_fingerprint_set_cmd_t *send_cmd = NULL;
  fpsensor_generic_response_t *resp = NULL;
  int32_t send_cmd_len = 0;
  int32_t resp_len = 0;

  struct QSEECom_ion_fd_info ion_fd_info;
  fpsensor_ion_info_t ion_handle;

  LOGD("%s begin", __func__);

  ion_handle.ion_fd = 0;
  ion_handle.ion_alloc_handle.handle = 0;

  retval = fpsensor_alloc_ion(&ion_handle, len);
  if (0 != retval)
  {
    LOGE("fpsensor_alloc_ion failed with retval = %d\n", retval);
    return FPSENSOR_TAC_ERROR_MEMORY;
  }

  memset(&ion_fd_info, 0, sizeof(struct QSEECom_ion_fd_info));

  ion_fd_info.data[0].fd = ion_handle.ifd_data_fd;
  ion_fd_info.data[0].cmd_buf_offset = sizeof(send_cmd->cmd_id);
  
  send_cmd = (fpsensor_delete_fingerprint_set_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;

  send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_delete_fingerprint_set_cmd_t));
  resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

  resp = (fpsensor_generic_response_t *) (tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len);

  send_cmd->cmd_id = FINGERPRINT_TEE_DELETE_FINGERPRINT_SET_CMD_ID;
  send_cmd->cmd_id ^= tac_handle->serialnum;
  
  send_cmd->fingerprint_set_key = (uint32_t) fingerprint_set_key;
  send_cmd->path = (uint32_t)(uintptr_t) ion_handle.ion_sbuffer;
  send_cmd->path_length = len;
  fpsensor_memcpy((unsigned char *)ion_handle.ion_sbuffer, path, len);

  resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;

  retval = QSEECom_send_modified_cmd(tac_handle->pQSEEComHandle,
                                     send_cmd,
                                     send_cmd_len,
                                     resp,
                                     resp_len,
                                     &ion_fd_info);
  
  if (0 != retval)
  {
    LOGE("QSEECom_send_modified_cmd failed with retval = %d\n", retval);
    fpsensor_dealloc_ion(&ion_handle);
    return FPSENSOR_TAC_ERROR_GENERAL;
  }

  fpsensor_dealloc_ion(&ion_handle);

  LOGD("%s end", __func__);

  return resp->status;
}
/** @brief:  Initialize TAC
 *  @param[in]	TZ application name
 *  @param[out]	pointer to TAC handle
 *  @return	zero on success or error code on failure
 */
int fpsensor_init(void **ppHandle)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_handle_internal_t* tac_handle = NULL;
  LOGD("%s begin", __func__);

  if (NULL == ppHandle)
  {
    	retval = FPSENSOR_TAC_ERROR_PARAMETER;
       LOGE("Invalid input parameter");
	return retval;
  }
  
tac_handle =  (fpsensor_handle_internal_t*) fpsensor_malloc(sizeof(fpsensor_handle_internal_t));

*ppHandle = NULL;

if (NULL != tac_handle)
{
  fpsensor_memset(tac_handle, 0, sizeof(fpsensor_handle_internal_t));
  *ppHandle = tac_handle;
}
else
{
  retval = FPSENSOR_TAC_ERROR_MEMORY;
  LOGE("No memory for handle");
  return retval;
}

  LOGD("%s end", __func__);
#if defined(_FPC_IMAGE_STORING_) && defined(FPC_TAC_DEBUG)

  _fpsensor_image_storage.raw = malloc(160 * 160);
  if (NULL == _fpsensor_image_storage.raw) {
      LOGE("Failed to allocate memory for the raw image.");
      return FPSENSOR_TAC_ERROR_MEMORY;
  }

  _fpsensor_image_storage.enhanced = malloc(160 * 160);
  if (NULL == _fpsensor_image_storage.enhanced) {
      LOGE("Failed to allocate memory for the enhanced image.");
      free(_fpsensor_image_storage.enhanced);
      return FPSENSOR_TAC_ERROR_MEMORY;
  }

  _fpsensor_image_storage.size = 160 * 160;

  fpc_ict_init();
#endif /* _FPC_IMAGE_STORING_ && FPC_TAC_DEBUG */

  return retval;
}

void fpc_km_tac_release_encapsulated_key(uint8_t* encapsulated_key)
{
    fpsensor_free(encapsulated_key);
}

int fpc_km_tac_get_encapsulated_key(uint8_t** encapsulated_key,
                              uint32_t* size_encapsulated_key)
{
    int retval = FPSENSOR_TAC_OK;
    *encapsulated_key = NULL;
    *size_encapsulated_key = 0;
    const uint32_t shared_buffer_size = 1024;

    struct QSEECom_handle* keymaster_handle = NULL;
    retval = QSEECom_start_app(&keymaster_handle,
                                  FPC_KEYMASTER_APP_PATH,
                                  FPC_KEYMASTER_APP_NAME,
                                  shared_buffer_size);


    if (retval) {
		LOGE("fail to start keymaster TA");
		return retval;
    }

    km_get_auth_token_req_t* command = (km_get_auth_token_req_t*)
            keymaster_handle->ion_sbuffer;

    uint32_t command_length = QSEECOM_ALIGN(sizeof(km_get_auth_token_req_t));
    km_get_auth_token_rsp_t* response = (km_get_auth_token_rsp_t*)
            (keymaster_handle->ion_sbuffer + command_length);


    command->cmd_id = KEYMASTER_GET_AUTH_TOKEN_KEY;
    command->auth_type = HW_AUTH_FINGERPRINT;

    uint32_t response_length = shared_buffer_size - command_length;

    retval = QSEECom_send_cmd(keymaster_handle,
                              command,
                              command_length,
                              response,
                              response_length);

    if (retval) {
        goto out;
    }

    *encapsulated_key = (uint8_t*)fpsensor_malloc(response->auth_token_key_len);
    if (*encapsulated_key == NULL) {
        retval = FPSENSOR_TAC_ERROR_MEMORY;
        goto out;
    }

    *size_encapsulated_key = response->auth_token_key_len;

    fpsensor_memcpy(*encapsulated_key,
                   ((uint8_t*) response) + response->auth_token_key_offset,
                   *size_encapsulated_key);

out:
    if (keymaster_handle) {
        QSEECom_shutdown_app(&keymaster_handle);
    }

    return retval;
}

int fpsensor_set_soter_key(const fpsensor_handle_internal_t* tac_handle)
{
	fpsensor_debug_image_cmd_t*soter_req = NULL;
	fpsensor_generic_response_t *soter_resp = NULL;

	int status = 0;
	int rc = 0;
	int rsa_cmd = 1;
	int update_share_data = 0;
	struct QSEECom_ion_fd_info ion_fd_info;
	struct QSEECom_handle *soter_handle;
	fpsensor_ion_info_t soter_ion_handle;
	int retval = FPSENSOR_TAC_OK;
	fpsensor_debug_image_cmd_t *send_cmd = NULL;
	fpsensor_generic_response_t *resp = NULL;
	int32_t send_cmd_len = 0;
	int32_t resp_len = 0;


	LOGD("%s begin", __func__);

	rc = QSEECom_start_app( &soter_handle, "/etc/firmware",
			"soter",  1024);
	if(rc)
	{
		LOGE("Fail to start soter TZAPP with errno: %u",errno);
		return -EINVAL;
	}	

	if(soter_handle == NULL)
	{
		LOGE("tzapp not started");
		return -EINVAL;
	}

	if(soter_handle->ion_sbuffer == NULL)
	{
		LOGE("ion_sbuffer not malloc");
		return -ENOMEM;
	}


	soter_ion_handle.ion_fd = 0;
	soter_ion_handle.ion_alloc_handle.handle = 0;
	if (fpsensor_alloc_ion(&soter_ion_handle, 1024) < 0) {
		LOGE("alipay ION allocation  failed");
		soter_ion_handle.ion_sbuffer = NULL;
		return -ENOMEM;
	}	

	memset(&ion_fd_info, 0, sizeof(struct QSEECom_ion_fd_info));

	ion_fd_info.data[0].fd = soter_ion_handle.ifd_data_fd;
	ion_fd_info.data[0].cmd_buf_offset = sizeof(send_cmd->cmd_id);


	send_cmd = (fpsensor_debug_image_cmd_t *) tac_handle->pQSEEComHandle->ion_sbuffer;

	send_cmd_len = QSEECOM_ALIGN(sizeof(fpsensor_debug_image_cmd_t));
	resp_len = QSEECOM_ALIGN(sizeof(fpsensor_generic_response_t));

	resp = (fpsensor_generic_response_t *) (tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len);

	send_cmd->cmd_id = FINGERPRINT_TEE_GET_ENCAPSULATED_KEY_CMD_ID;
	send_cmd->cmd_id ^= tac_handle->serialnum;
	send_cmd->image_data = (uint32_t)(uintptr_t) soter_ion_handle.ion_sbuffer;
	send_cmd->image_len = 1024;

	resp->status = FPSENSOR_TAC_ERROR_NO_RESPONSE;
	QSEECom_set_bandwidth(tac_handle->pQSEEComHandle,true);

	retval = QSEECom_send_modified_cmd(tac_handle->pQSEEComHandle,
			send_cmd,
			send_cmd_len,
			resp,
			resp_len,
			&ion_fd_info);


	if (0 != retval)
	{
		LOGE("QSEECom_send_modified_cmd failed with retval = %d\n", retval);
		retval = -FPSENSOR_TAC_ERROR_GENERAL;
		goto err;
	}

	if (resp->status)
	{
		LOGD("get hmac key fail with status:%d\n",status);
		retval = -FPSENSOR_TAC_ERROR_GENERAL;
		goto err;
	}
	LOGD("get hmac key success with size:%d\n",resp->data);
	if (resp->data <= 0)
	{
		LOGD("hmac key len error");
		retval = -FPSENSOR_TAC_ERROR_GENERAL;
		goto err;
	}
	soter_req=(fpsensor_debug_image_cmd_t *)soter_handle->ion_sbuffer;
	soter_resp = (fpsensor_generic_response_t *)(soter_handle->ion_sbuffer+QSEECOM_ALIGN(sizeof(*soter_req)));

	memset(&ion_fd_info, 0, sizeof(struct QSEECom_ion_fd_info));

	ion_fd_info.data[0].fd = soter_ion_handle.ifd_data_fd;
	ion_fd_info.data[0].cmd_buf_offset = sizeof(soter_req->cmd_id);
	soter_req->cmd_id = KEYMASTER_SET_ENCAPSULATED_KEY;
	soter_req->image_data = (uint32_t)(uintptr_t)soter_ion_handle.ion_sbuffer;
	soter_req->image_len = resp->data;
	soter_resp->status = -1;
	retval=QSEECom_send_modified_cmd(soter_handle, soter_req,
			QSEECOM_ALIGN(sizeof(*soter_req)), soter_resp,QSEECOM_ALIGN(sizeof(*soter_resp)),&ion_fd_info);

	QSEECom_set_bandwidth(tac_handle->pQSEEComHandle,false);
	if (0 != retval)
	{
		LOGE("QSEECom_send_modified_cmd failed with retval = %d\n", retval);

		retval = -FPSENSOR_TAC_ERROR_GENERAL;
		goto err;
	}

	if (resp->status)
	{
		LOGD("set hmac key fail with status:%d\n",status);
		retval = -FPSENSOR_TAC_ERROR_GENERAL;
		goto err;
	}

	LOGD("send_cmd success. soter_resp->len is %d",soter_resp->data);
err:
	fpsensor_dealloc_ion(&soter_ion_handle);

	if(soter_handle)
	{
		QSEECom_shutdown_app( &soter_handle);	
		soter_handle = NULL;
	}
	return retval;
}

/** @brief:  Open link to TZ application
 *  @param[in]	TAC handle
 *  @return	zero on success or error code on failure
 */
int fpsensor_open(void *pHandle)
{
  int retval = FPSENSOR_TAC_OK;
  int qsee_retval = 0;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  LOGD("%s begin", __func__);
  uint8_t* encapsulated_key = NULL;
  uint32_t size_encapsulated_key = 0;
  if ((NULL == tac_handle))
  {
    retval = FPSENSOR_TAC_ERROR_PARAMETER;
    LOGE("Invalid input parameters");
    goto err;
  }  	

LOGI("Application path: %s", FPC_TAC_APPLICATION_PATH);
LOGI("Application name: %s", FPC_TAC_APPLICATION_NAME);

qsee_retval = QSEECom_start_app(&(tac_handle->pQSEEComHandle),
                                FPC_TAC_APPLICATION_PATH,
                                FPC_TAC_APPLICATION_NAME,
                                1024); // TODO: Change

if (qsee_retval)
{
  LOGE("QSEECom_start_app failed, qsee_retval = %d ", qsee_retval);
  retval = FPSENSOR_TAC_ERROR_APP_NOT_FOUND;
  goto err; 
}	

if (NULL == tac_handle->pQSEEComHandle)
{
	LOGE("QSEECom_start_app failed, pQSEEComHandle is NULL");
	retval = FPSENSOR_TAC_ERROR_GENERAL;
  goto err; 
	
}


if (NULL == tac_handle->pQSEEComHandle->ion_sbuffer)
{
  LOGE("QSEECom_start_app failed, ion_sbuffer is NULL");
  retval = FPSENSOR_TAC_ERROR_GENERAL;
  goto err; 
  
}

  LOGI("ion_sbuffer: %p", tac_handle->pQSEEComHandle->ion_sbuffer);

  qsee_retval = fpc_km_tac_get_encapsulated_key(&encapsulated_key,
                                                &size_encapsulated_key);

  if (0 == qsee_retval) {
    qsee_retval = fpsensor_send_encapsulated_key(tac_handle, encapsulated_key,
                                                size_encapsulated_key);

	fpsensor_set_soter_key(tac_handle);
    }
    else
    {
      LOGE("%s failed to setup hw auth token key: %d", __func__, qsee_retval);
    }
    
err:
  LOGD("%s end", __func__);
  if(encapsulated_key)
  	fpc_km_tac_release_encapsulated_key(encapsulated_key);
  
  return retval;
}

/** @brief:  Close link to TZ application free resources
 *  @param[in]	TAC handle
 *  @return	zero on success or error code on failure
 */
int fpsensor_close(void *pHandle)
{
  int retval = FPSENSOR_TAC_OK;
  int qsee_retval = 0;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  LOGD("%s begin", __func__);

  if ((NULL != tac_handle))
  {
    qsee_retval = QSEECom_shutdown_app(&(tac_handle->pQSEEComHandle));
	tac_handle->pQSEEComHandle = NULL;
    if (0 != qsee_retval)
    {
      LOGE("QSEECom_shutdown_app failed, qsee_retval = %d ", qsee_retval);
      retval = FPSENSOR_TAC_ERROR_GENERAL;
    }
  }
  else
  {
    retval = FPSENSOR_TAC_ERROR_PARAMETER;
    LOGE("Invalid input parameter");
  }


  fpsensor_free(tac_handle->user_tpl_storage_path);
  fpsensor_free(tac_handle->global_tpl_storage_path);

  LOGD("%s end", __func__);

  return retval;
}
int fpsensor_set_active_fingerprint_set(void *pHandle, int32_t fingerprint_set_key)
{
    int retval = FPSENSOR_TAC_OK;
    fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

    LOGD("%s begin", __func__);

    retval = fpsensor_send_cmd(tac_handle, FINGERPRINT_TEE_SET_ACTIVE_FINGERPRINT_SET_CMD_ID, (uint32_t)fingerprint_set_key);

    if (FPSENSOR_TAC_OK != retval)
    {
      LOGE("fpsensor_send_cmd failed with retval: %d", retval);
    }

    LOGD("%s end", __func__);

    return retval;
}

int fpsensor_delete_fingerprint_set(void *pHandle,
                                   int32_t fingerprint_set_key,
                                   const char* path,
                                   uint32_t path_len)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  LOGD("%s begin", __func__);
  LOGD("Trying to delete template DB: '%s' user: %u", path, (uint32_t)fingerprint_set_key);

  retval = fpsensor_send_delete_fingerprint_set(tac_handle, fingerprint_set_key, path, path_len);

  if (FPSENSOR_TAC_OK != retval)
  {
    LOGE("fpsensor_send_delete_fingerprint_set failed with retval: %d", retval);
  }

  if (tac_handle->tpl_storage_type == TEMPLATE_STORAGE_TYPE_HLOS)
  {
    // Delete template db file
    LOGD("Trying to delete template DB: %s", path);
    retval = fpsensor_delete_template_db_from_hlos_fs(path);
    if (FPSENSOR_TAC_OK != retval)
    {
      LOGE("fpsensor_delete_template_db_from_hlos_fs failed with retval: %d", retval);
    }
  }

  LOGD("%s end", __func__);

  return retval;
}
int fpsensor_get_template_id_from_index(void *pHandle, uint32_t index, uint32_t* data)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  LOGD("%s begin", __func__);

  /* retval != 0 doesn't have to mean error in this case */
  retval = fpsensor_send_get_template_id_from_index_cmd(tac_handle, index , data);

  LOGD("%s end", __func__);

  return retval;
}

int fpsensor_get_indices(void *pHandle,
                    uint32_t* indices, uint32_t* index_count)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  retval = fpsensor_send_get_indices_cmd(tac_handle, indices, index_count);
  if (FPSENSOR_TAC_OK != retval)
  {
    LOGE("fpsensor_send_get_indices_cmd failed with retval: %d", retval);
  }

  LOGD("%s end", __func__);

  return retval;
}

int fpsensor_get_template_count(void *pHandle,
                         uint32_t* count)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  LOGD("%s begin", __func__);

  retval = fpsensor_send_get_uint32_cmd(tac_handle, FINGERPRINT_TEE_GET_TEMPLATE_COUNT_CMD_ID, count);
  if (FPSENSOR_TAC_OK != retval)
  {
    LOGE("fpsensor_send_get_uint32_cmd failed with retval: %d", retval);
  }

  LOGD("%s end", __func__);

  return retval;
}

int fpsensor_delete_template(void *pHandle, uint32_t index)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  LOGD("%s begin", __func__);

  retval = fpsensor_send_cmd(tac_handle, FINGERPRINT_TEE_DELETE_TEMPLATE_CMD_ID, index);
  if (FPSENSOR_TAC_OK != retval)
  {
    LOGE("fpsensor_send_cmd failed with retval: %d", retval);
  }

  LOGD("%s end", __func__);

  return retval;
}

int fpsensor_get_auth_data(void *pHandle, const uint8_t* nonce, const char* secAppName, uint8_t *buf, int32_t *len)
{
  int retval = FPSENSOR_TAC_OK;
  fpsensor_auth_data_t AuthData;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  LOGD("%s begin", __func__);

  retval = fpsensor_send_get_auth_data_cmd(tac_handle, nonce, secAppName, &AuthData);
  if (FPSENSOR_TAC_OK != retval)
  {
    LOGE("fpsensor_send_get_auth_data_cmd failed with retval: %d", retval);
    return retval;
  }
  if(*len < sizeof(AuthData))
  {
	LOGE("Error:len=%d is less then sizeof AuthData %d",*len,sizeof(AuthData));
  }
  else
	  *len = sizeof(AuthData);

  memcpy(buf,(char *)&AuthData,*len);
  LOGD("%s end", __func__);

  return retval;
}

static int _fpsensor_store_template_db(fpsensor_handle_internal_t* tac_handle, fpsensor_db_type_t db_type)
{
  int retval = FPSENSOR_TAC_OK;
  int qsee_retval = 0;
  uint32_t size = 0;

#ifdef TAC_TIME_MEASUREMENTS
  struct timeval stop, start, delta;
  int time = 0;

  gettimeofday(&start, NULL);
#endif

  /* Make sure the crypto block in QSEE runs in full speed, for best SFS performance */
  qsee_retval = QSEECom_set_bandwidth(tac_handle->pQSEEComHandle, true);
  if (qsee_retval != 0)
  {
    LOGE("QSEECom_set_bandwidth failed with retval: %d", qsee_retval);
    retval = FPSENSOR_TAC_ERROR_GENERAL;
    goto err;
  }

  if (tac_handle->tpl_storage_type == TEMPLATE_STORAGE_TYPE_HLOS)
  {
    retval = fpsensor_send_get_template_size_cmd(tac_handle, db_type, &size);
    if (FPSENSOR_TAC_OK != retval)
    {
      LOGE("fpsensor_send_get_uint32_cmd failed with retval: %d", retval);
    }

    if (FPSENSOR_TAC_OK == retval) {
      retval = fpsensor_send_read_template_cmd(tac_handle, size, db_type);
      if (FPSENSOR_TAC_OK != retval)
      {
        LOGE("fpsensor_update_template failed with retval: %d", retval);
      }
    }
  }
  else if (tac_handle->tpl_storage_type == TEMPLATE_STORAGE_TYPE_TZ)
  {
    retval = fpsensor_send_cmd(tac_handle, FINGERPRINT_TEE_STORE_TEMPLATE_DB_CMD_ID, db_type);
    if (FPSENSOR_TAC_OK != retval)
    {
      LOGE("fpsensor_send_cmd failed with retval: %d", retval);
    }
  }

  qsee_retval = QSEECom_set_bandwidth(tac_handle->pQSEEComHandle, false);
  if (qsee_retval != 0)
  {
    LOGE("QSEECom_set_bandwidth failed with retval: %d", qsee_retval);
    retval = FPSENSOR_TAC_ERROR_GENERAL;
    goto err;
  }

#ifdef TAC_TIME_MEASUREMENTS
  gettimeofday(&stop, NULL);

  timersub(&stop, &start, &delta);
  time = delta.tv_sec * USECS_PER_SECOND + delta.tv_usec;
  LOGE("Store template db took %d ms\n", time / MSECS_PER_SECOND);
#endif

err:
  return retval;
}

int fpsensor_store_template_db(void *pHandle)
{
  int status;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;
  char global_backup[128];
  char user_backup[128];

  LOGD("%s begin", __func__);

  sprintf(global_backup, "%s.bak", tac_handle->global_tpl_storage_path);
  sprintf(user_backup, "%s.bak", tac_handle->user_tpl_storage_path);

  rename(tac_handle->global_tpl_storage_path, global_backup);
  rename(tac_handle->user_tpl_storage_path, user_backup);

  status = _fpsensor_store_template_db(tac_handle, DB_TYPE_GLOBAL);
  if (FPSENSOR_TAC_OK != status)
  {
    LOGE("Failed to write global db");
    goto err;
  }

  status = _fpsensor_store_template_db(tac_handle, DB_TYPE_USER);
  if (FPSENSOR_TAC_OK != status)
  {
    LOGE("Failed to write user db");
    goto err;
  }

  remove(global_backup);
  remove(user_backup);

  LOGD("%s end", __func__);

  return FPSENSOR_TAC_OK;

err:
  rename(global_backup, tac_handle->global_tpl_storage_path);
  rename(user_backup, tac_handle->user_tpl_storage_path);

  return FPSENSOR_TAC_ERROR_GENERAL;
}


bool _fpsensor_set_template_db_path(void *pHandle,
                                   const char* path,
                                   uint32_t len,
                                   fpsensor_db_type_t db_type) {
  char* temp;
  char** current_path;

  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  switch (db_type) {
  case DB_TYPE_GLOBAL:
    current_path = &tac_handle->global_tpl_storage_path;
    break;
  case DB_TYPE_USER:
    current_path = &tac_handle->user_tpl_storage_path;
    break;
  default:
    LOGE("%s: wrong DB type set %u", __func__, db_type);
    return false;
  }

  if (*current_path != NULL &&
         (0 == strcmp(*current_path, path)))
  {
      /* Don't set the same path twice */
    return false;
  }
  else
  {
    temp = (char *)malloc(len);
    if (NULL == temp)
    {
        LOGE("Failed to allocate memory for template database");
        return false;
    }

    if (NULL != *current_path)
    {
        free(*current_path);
    }

    *current_path = temp;
    memcpy(*current_path, path, len);
    return true;
  }
}


static int fpsensor_load_template_db(void *pHandle,
                                    const char* path,
                                    uint32_t path_len,
                                    fpsensor_db_type_t db_type)
{
  int retval = FPSENSOR_TAC_OK;
  bool new_db = false;
  int qsee_retval = 0;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

#ifdef TAC_TIME_MEASUREMENTS
  struct timeval stop, start, delta;
  int time = 0;
#endif

#ifdef TAC_TIME_MEASUREMENTS
  gettimeofday(&start, NULL);
#endif

  /* Make sure the crypto block in QSEE runs in full speed, for best SFS performance */
  qsee_retval = QSEECom_set_bandwidth(tac_handle->pQSEEComHandle, true);
  if (FPSENSOR_TAC_OK != qsee_retval)
  {
    LOGE("QSEECom_set_bandwidth failed with retval: %d", qsee_retval);
    retval = FPSENSOR_TAC_ERROR_GENERAL;
    goto err;
  }

  if (tac_handle->tpl_storage_type == TEMPLATE_STORAGE_TYPE_HLOS)
  {
    new_db = _fpsensor_set_template_db_path(tac_handle, path, path_len, db_type);
    if (!new_db) {
        LOGD("Path is the same as the old one.");
        goto err;
    }
    LOGD("Database set to %s", path);
    retval = fpsensor_send_template_db(tac_handle, db_type);
    if (FPSENSOR_TAC_OK != retval)
    {
      LOGE("fpsensor_send_template_db failed with retval: %d", retval);
    }
  }
  else if (tac_handle->tpl_storage_type == TEMPLATE_STORAGE_TYPE_TZ)
  {
    retval = fpsensor_send_set_template_db_path(tac_handle, path, path_len, db_type);
    if (FPSENSOR_TAC_OK != retval)
    {
      LOGE("fpsensor_send_set_template_db path failed with retval: %d", retval);
    }

    retval = fpsensor_send_cmd(tac_handle, FINGERPRINT_TEE_LOAD_TEMPLATE_DB_CMD_ID, db_type);
    if (FPSENSOR_TAC_OK != retval)
    {
      LOGE("fpsensor_send_cmd failed with retval: %d", retval);
    }

    new_db = _fpsensor_set_template_db_path(tac_handle, path, path_len, db_type);
    if (!new_db)
    {
      LOGE("Failed to store new database path.");
    }
  }

  qsee_retval = QSEECom_set_bandwidth(tac_handle->pQSEEComHandle, false);
  if (FPSENSOR_TAC_OK != retval)
  {
    LOGE("QSEECom_set_bandwidth failed with retval: %d", qsee_retval);
    retval = FPSENSOR_TAC_ERROR_GENERAL;
    goto err;
  }

#ifdef TAC_TIME_MEASUREMENTS
  gettimeofday(&stop, NULL);

  timersub(&stop, &start, &delta);
  time = delta.tv_sec * USECS_PER_SECOND + delta.tv_usec;
  LOGE("Load template db took %d ms\n", time / MSECS_PER_SECOND);
#endif

err:
  return retval;
}

int fpsensor_load_global_db(void *pHandle, const char* path, uint32_t path_len)
{
  int retval;
  LOGD("%s begin", __func__);

  retval = fpsensor_load_template_db(pHandle, path, path_len, DB_TYPE_GLOBAL);

  LOGD("%s end", __func__);
  return retval;
}

int fpsensor_load_user_db(void *pHandle, const char* path, uint32_t path_len)
{
  int retval;
  LOGD("%s begin", __func__);

  retval = fpsensor_load_template_db(pHandle, path, path_len, DB_TYPE_USER);

  LOGD("%s end", __func__);
  return retval;
}


int fpsensor_is_user_id_valid(void *pHandle, uint64_t user)
{

  LOGD("%s begin", __func__);

  if (pHandle == NULL)
  {
    return FPSENSOR_TAC_ERROR_PARAMETER;
  }

  const fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  int retval = fpsensor_send_cmd64( tac_handle, FINGERPRINT_TEE_IS_VALID_USER_CMD_ID, user);

  if (retval != FPSENSOR_TAC_OK)
  {
    LOGE("fpsensor_send_cmd64 failed with retval: %d", retval);
  }

  LOGD("%s end", __func__);

  return retval;
}

int fpsensor_get_template_db_id(void *pHandle, uint64_t* id) {
  int retval = FPSENSOR_TAC_OK;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  LOGD("%s begin", __func__);

  retval = fpsensor_send_get_uint64_cmd(tac_handle, FINGERPRINT_TEE_GET_TEMPLATE_DB_ID_CMD_ID, id);
  if (FPSENSOR_TAC_OK != retval)
  {
    LOGE("fpsensor_send_get_uint64_cmd failed with retval: %d", retval);
  }

  LOGD("%s end", __func__);

  return retval;
}

int fpsensor_get_hw_auth_challenge(void *pHandle,
                                  uint64_t* challenge) {
  int retval = FPSENSOR_TAC_OK;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  LOGD("%s begin", __func__);

  retval = fpsensor_send_get_uint64_cmd(tac_handle, FINGERPRINT_TEE_GET_HW_AUTH_CHALLENGE_CMD_ID, challenge);
  if (FPSENSOR_TAC_OK != retval)
  {
    LOGE("fpsensor_send_get_uint64_cmd failed with retval: %d", retval);
  }

  LOGD("%s end", __func__);

  return retval;
}

int fpsensor_set_hw_auth_challenge(void *pHandle,
                                  uint64_t challenge) {
  int retval = FPSENSOR_TAC_OK;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  LOGD("%s begin", __func__);

  retval = fpsensor_send_set_uint64_cmd(tac_handle, FINGERPRINT_TEE_SET_HW_AUTH_CHALLENGE_CMD_ID, challenge);
  if (FPSENSOR_TAC_OK != retval)
  {
    LOGE("fpsensor_send_set_uint64_cmd failed with retval: %d", retval);
  }

  LOGD("%s end", __func__);

  return retval;
}

int fpsensor_validate_auth_challenge(void *pHandle,
                                    const uint8_t* auth_token,
                                    uint32_t size_token) {
  int retval = FPSENSOR_TAC_OK;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  LOGD("%s begin", __func__);

  retval = fpsensor_send_set_arbitrary_data_cmd(tac_handle,
          FINGERPRINT_TEE_VALIDATE_AUTH_CHALLENGE_CMD_ID,(uint8_t*) auth_token, size_token);
  if (FPSENSOR_TAC_OK != retval)
  {
    LOGE("fpsensor_send_set_arbitrary_data_cmd failed with retval: %d", retval);
  }

  /* remap timeout error */
  if (FPC_LIB_ERROR_TOO_SLOW == retval)
  {
    retval = FPSENSOR_TAC_ERROR_TIMEOUT;
  }

  LOGD("%s end", __func__);

  return retval;
}

int fpsensor_get_hw_auth_token(void *pHandle,
                              uint8_t* auth_token,
                              uint32_t size_auth_token) {
  int retval = FPSENSOR_TAC_OK;
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

  LOGD("%s begin", __func__);

  retval = fpsensor_send_get_arbitrary_data_cmd(tac_handle,
          FINGERPRINT_TEE_GET_HW_AUTH_TOKEN_CMD_ID, auth_token, size_auth_token);
  if (FPSENSOR_TAC_OK != retval)
  {
    LOGE("fpsensor_send_get_arbitrary_data_cmd failed with retval: %d", retval);
  }

  LOGD("%s end", __func__);

  return retval;
}
                            
int32_t fpsensor_send_cmd(void *pHandle, uint32_t cmdID, uint32_t data, uint32_t* result)
{    
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

	struct qseecom_cmd_data *req = NULL;
	struct qseecom_rsp_data *rsp = NULL;
	int status;
	int32_t send_cmd_len = 0;


	req=(struct qseecom_cmd_data *)tac_handle->pQSEEComHandle->ion_sbuffer;
	send_cmd_len = QSEECOM_ALIGN(sizeof(*req));
	rsp = (struct qseecom_rsp_data *)(tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len);
	req->cmd_id = cmdID;
	req->cmd_id ^= tac_handle->serialnum;
	req->data = data;
	rsp->status = -1;
	status=QSEECom_send_cmd(tac_handle->pQSEEComHandle, req,
	        send_cmd_len, rsp,QSEECOM_ALIGN(sizeof(*rsp)));

	if (status)
	{
	    LOGD("Fail to send cmdID %d with errno: %u", cmdID, errno);
	    return status;
	}

	if(result)
		*result = rsp->data;

	if (rsp->status)
	{
	       LOGD("Fail to do cmdID %d with status: %d", cmdID, rsp->status);
	       return rsp->status;
	}
	
	return 0;

}	

int32_t fpsensor_send_cmd_rsp(void *pHandle, uint32_t cmdID, uint32_t data, struct qseecom_rsp_buf **prsp)
{    
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

	struct qseecom_cmd_data *req = NULL;
	struct qseecom_rsp_buf *rsp = NULL;
	int status;
	int32_t send_cmd_len = 0;

	req=(struct qseecom_cmd_data *)tac_handle->pQSEEComHandle->ion_sbuffer;
	send_cmd_len = QSEECOM_ALIGN(sizeof(*req));
	rsp = (struct qseecom_rsp_buf *)(tac_handle->pQSEEComHandle->ion_sbuffer + send_cmd_len);
	*prsp = rsp;
	req->cmd_id = cmdID;
	req->cmd_id ^= tac_handle->serialnum;
	req->data = data;
	rsp->status = -1;
	status=QSEECom_send_cmd(tac_handle->pQSEEComHandle, req,
	        send_cmd_len, rsp,QSEECOM_ALIGN(sizeof(*rsp)));

	if (status)
	{
	    LOGD("Fail to send CLIENT_CMD16_FB_REMOVE_TEMPLATE with errno: %u",errno);
	    return status;
	}

	if (rsp->status)
	{
	       LOGD("Fail to do cmdID %d with errno: %u", cmdID, errno);
	       return rsp->status;
	}
	
	return 0;
}	

int fpsensor_save_bitmap(const char* FilePath, unsigned char *pData,
		int row, int colume)
{
	/*Check parameter*/
    int fd = 0;
    unsigned char grayBitmapHeader[1078] = { 0 };
	unsigned char pad[4] = {0};
    int colume_t = 0;
    int row_t = 0;
    int i =0;
	if (NULL == pData)
	{
		return -1;
	}
	fd = open(FilePath,O_RDWR| O_CREAT,0777);
	if (0 == fd)
	{
		return -1;
	}

	colume_t = (colume + 3) & 0xFC;
	row_t = (row + 3) & 0xFC;
	grayBitmapHeader[0] = 0x42;
	grayBitmapHeader[1] = 0x4d;
	grayBitmapHeader[2] = 0x36;
	grayBitmapHeader[3] = 0x28;
	grayBitmapHeader[4] = 0x00;
	grayBitmapHeader[5] = 0x00;
	grayBitmapHeader[6] = 0x00;
	grayBitmapHeader[7] = 0x00;
	grayBitmapHeader[8] = 0x00;
	grayBitmapHeader[9] = 0x00;
	grayBitmapHeader[10] = 0x36;
	grayBitmapHeader[11] = 0x04;
	grayBitmapHeader[12] = 0x00;
	grayBitmapHeader[13] = 0x00;
	grayBitmapHeader[14] = 0x28;
	grayBitmapHeader[15] = 0x00;
	grayBitmapHeader[16] = 0x00;
	grayBitmapHeader[17] = 0x00;

	grayBitmapHeader[18] = (colume_t & 0xFF);
	grayBitmapHeader[19] = (colume_t >> 8 & 0xFF);
	grayBitmapHeader[20] = (colume_t >> 16 & 0xFF);
	grayBitmapHeader[21] = (colume_t >> 24 & 0xFF);

	grayBitmapHeader[22] = (row_t & 0xFF);
	grayBitmapHeader[23] = (row_t >> 8 & 0xFF);
	grayBitmapHeader[24] = (row_t >> 16 & 0xFF);
	grayBitmapHeader[25] = (row_t >> 24 & 0xFF);

	grayBitmapHeader[26] = 0x01;
	grayBitmapHeader[27] = 0x00;

	grayBitmapHeader[28] = 0x08;
	grayBitmapHeader[29] = 0x00;

	for (i = 0; i < 256; i++)
	{
		grayBitmapHeader[54 + i * 4] = i;
		grayBitmapHeader[54 + i * 4 + 1] = i;
		grayBitmapHeader[54 + i * 4 + 2] = i;
	}
	write(fd,(char *) grayBitmapHeader, 1078);


	/*Be careful , bitmap save turn is buttom to top*/	
	for (i = 0; i < row; i++)
	{
		write(fd,(char *) (pData + (row - i - 1) * colume),
				colume);
		if(colume_t > colume)
			write(fd,(char *)pad, colume_t - colume);
	}
	close(fd);
	return 0;
}

static int32_t fpsensor_probe_sensor(fingerprint_data_t* device)
{    
	struct qseecom_rsp_buf *rsp = NULL;
    char sensor_name[32]={0};	
    int32_t status = 0;
	unsigned int speed = 1000000;
	
	status = ioctl(device->sysfs_fd,FPSENSOR_IOC_GET_SENSOR_NAME,sensor_name);
	if(status < 0)
	{
	        ALOGE("Failed to get sensor name %s func with error %d: %s\n",__func__,errno,strerror(errno));
	        return -EFAULT;
	}	
	
	if(!strcmp(sensor_name,"fpc1021") || !strcmp(sensor_name,"fpc1022") ||!strcmp(sensor_name,"fpc1140") )
	{
		ALOGE("detect %s sucess,use fpc functions",sensor_name);
		speed = 5000000;
		device->device_init= fpc_device_init;
	}
	else if(!strcmp(sensor_name,"goodix318m") || !strcmp(sensor_name,"goodix518m"))
	{
		ALOGE("detect %s sucess,use goodix functions",sensor_name);
		speed = 1000000;
		//device->device_init= goodix_device_init;
	}
	else 
	{
		ALOGE("unkown %s",sensor_name);
		device->device_init= NULL;
		status = -1;
	}

	ioctl(device->sysfs_fd,FPSENSOR_IOC_HW_RESET,NULL);
	status = ioctl(device->sysfs_fd,FPSENSOR_IOC_SETCLKRATE,&speed);
	ioctl(device->sysfs_fd,FPSENSOR_IOC_HW_PREPARE,NULL);

	status = fpsensor_send_cmd((fpsensor_handle_internal_t*)device->tac_handle, FINGERPRINT_TEE_PROBE_SENSOR_CMD_ID, 0);

	if (FPSENSOR_TAC_OK != status)
	{
	    ALOGE("fpsensor_send_cmd failed with retval: %d", status);
	}

	ioctl(device->sysfs_fd,FPSENSOR_IOC_HW_UNPREPARE,NULL);
    if (status)
    {
        LOGD("Fail to do probe sensor with status: %d",status);
		return status;
    }
	device->device_init(device);
	return status;
}

static int32_t fpsensor_setFingerprintName(void *pHandle, char* name, int32_t id)
{    
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

    struct qseecom_cmd_buf *req = NULL;
    struct qseecom_rsp_data *rsp = NULL;
    int status;
	int req_len = QSEECOM_ALIGN(sizeof(*req) + NAME_STR_LENGTH);

    LOGD("setname id=%d,name is %s,strlen=%d\n",id,name,strlen(name));

    req=(struct qseecom_cmd_buf *)tac_handle->pQSEEComHandle->ion_sbuffer;
    rsp = (struct qseecom_rsp_data *)(tac_handle->pQSEEComHandle->ion_sbuffer + req_len);
	memset(req,0,req_len);
    req->cmd_id = FINGERPRINT_TEE_SET_NAME_CMD_ID;
	req->cmd_id ^= tac_handle->serialnum;
    req->data   = id;
    req->len   = strlen(name);
    strncpy(req->sbuf,name,strlen(name));

    status=QSEECom_send_cmd(tac_handle->pQSEEComHandle, req,req_len, rsp,QSEECOM_ALIGN(sizeof(*rsp)));

    if (status)
    {
        LOGD("Fail to send CLIENT_CMD50_FB_CAC with errno: %u",errno);
        return status;
    }

    if (rsp->status)
    {
        LOGD("Fail to do calibration with errno: %u",errno);
           return rsp->status;
    }

    LOGD("setname id=%d,rsp id=%d\n",id,rsp->data);
    return status;

}

static int32_t fpsensor_getFingerprintName(void *pHandle, char* name, int32_t id)
{    
  fpsensor_handle_internal_t* tac_handle = (fpsensor_handle_internal_t*) pHandle;

    struct qseecom_rsp_buf *rsp = NULL;
    int32_t status;


    status = fpsensor_send_cmd_rsp(pHandle, FINGERPRINT_TEE_GET_NAME_CMD_ID, id, &rsp);

    if (status)
    {
        LOGD("Fail to do get fpname with status: %d",status);
           return 0;
    }

    strncpy(name,rsp->sbuf,rsp->len);
    LOGD("id=%d,name is %s,strlen=%d\n",id,name,rsp->len);
    return rsp->len;

}
                                            

	  
int32_t fpsensor_device_init(fingerprint_data_t *device)
{
	fpsensor_handle_internal_t* tac_handle = NULL;
	int rc = 0;
	char property[PROPERTY_VALUE_MAX]="Unkown fp module";
	rc = fpsensor_init(&device->tac_handle);
	if(rc)
	{
		ALOGE("fpsensor_init fail with error %d",rc);
		return -ENOMEM;
	 }
	
	tac_handle = (fpsensor_handle_internal_t*) device->tac_handle;
	device->probe_sensor = fpsensor_probe_sensor;
	device->set_name= fpsensor_setFingerprintName;
	device->get_name= fpsensor_getFingerprintName;
	device->set_hw_auth_challenge= fpsensor_set_hw_auth_challenge;
	device->get_hw_auth_challenge=fpsensor_get_hw_auth_challenge;
	device->validate_auth_challenge= fpsensor_validate_auth_challenge;
	device->get_hw_auth_token=fpsensor_get_hw_auth_token;
	device->get_auth_data=fpsensor_get_auth_data;
	device->load_global_db=fpsensor_load_global_db;
	device->load_user_db=fpsensor_load_user_db;
	device->get_template_count=fpsensor_get_template_count;
	device->get_indices=fpsensor_get_indices;
	device->get_template_id_from_index=fpsensor_get_template_id_from_index;
	device->get_template_db_id=fpsensor_get_template_db_id;
	device->store_template_db=fpsensor_store_template_db;
	device->delete_template=fpsensor_delete_template;
	device->set_active_fingerprint_set=fpsensor_set_active_fingerprint_set;

	device->sysfs_fd = open(FINGERPRINT_SENSOR_PATH,O_RDONLY);
	if(device->sysfs_fd<0)
	{
	    ALOGE("Failed to open %s with error %d: %s\n",FINGERPRINT_SENSOR_PATH,errno,strerror(errno));
		rc  = property_set("persist.yulong.fpModuleName", "No Sensor");
	    return -ENODEV;
	}
	
	rc = ioctl(device->sysfs_fd,FPSENSOR_IOC_GET_MODULE_NAME,property);
	if(rc < 0)
	{
        ALOGE("Failed to read fingerprint module's name in %s func with error %d: %s\n",__func__,errno,strerror(errno));
	}
	ALOGE("fingerprint module's name: %s\n",property);
	rc  = property_set("persist.yulong.fpModuleName", property);
	if (rc < 0) {
        	ALOGE("%s: Error while updating fpModuleName property: %d\n", __func__, rc);
	}
	rc = ioctl(device->sysfs_fd,FPSENSOR_IOC_GET_SERIAL_NUM,&tac_handle->serialnum);
	if(rc < 0)
	{
        ALOGE("Failed to read serialnum in %s func with error %d: %s\n",__func__,errno,strerror(errno));
        return -EFAULT;
	}
	ioctl(device->sysfs_fd,FPSENSOR_IOC_HW_UNPREPARE,NULL);
	return 0;
}
