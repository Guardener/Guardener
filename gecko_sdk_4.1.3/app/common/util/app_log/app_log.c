/***************************************************************************//**
 * @file
 * @brief Application log source file
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#ifdef HOST_TOOLCHAIN
#include <time.h>
#if defined(POSIX) && POSIX == 1
#include <sys/time.h>
#else
#include <windows.h>
#endif // defined(POSIX) && POSIX == 1

#define SL_WEAK
#else // HOST_TOOLCHAIN
#include "em_common.h"
#endif // HOST_TOOLCHAIN

#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif // SL_COMPONENT_CATALOG_PRESENT

#include "sl_status.h"
#include "app_log.h"
#include "app_log_config.h"
#include <string.h>
#include <stdio.h>

#ifdef SL_CATALOG_SLEEPTIMER_PRESENT
#include "sl_sleeptimer.h"
#endif // SL_CATALOG_SLEEPTIMER_PRESENT

// -----------------------------------------------------------------------------
// Global variables

/// IOStream for logging
sl_iostream_t * app_log_iostream = NULL;

// -----------------------------------------------------------------------------
// Local variables

#if APP_LOG_COUNTER_ENABLE == 1
/// Counter for logging
static uint32_t counter = 0;
#endif // APP_LOG_COUNTER_ENABLE

/// Simple filter status
static bool level_filter_enabled = APP_LOG_LEVEL_FILTER_ENABLE;

/// Simple filter for logging
static uint8_t level_filter_threshold = APP_LOG_LEVEL_FILTER_THRESHOLD;

/// Mask status
static bool level_mask_enabled = APP_LOG_LEVEL_MASK_ENABLE;

/// Mask for logging
static uint8_t level_mask =
  (APP_LOG_LEVEL_MASK_CRITICAL << APP_LOG_LEVEL_CRITICAL)
  | (APP_LOG_LEVEL_MASK_ERROR << APP_LOG_LEVEL_ERROR)
  | (APP_LOG_LEVEL_MASK_WARNING << APP_LOG_LEVEL_WARNING)
  | (APP_LOG_LEVEL_MASK_INFO << APP_LOG_LEVEL_INFO)
  | (APP_LOG_LEVEL_MASK_DEBUG << APP_LOG_LEVEL_DEBUG);

// -----------------------------------------------------------------------------
// Public functions

/***************************************************************************//**
 * Checking log level
 ******************************************************************************/
bool _app_log_check_level(uint8_t level)
{
  bool ret = true;
  if (level_filter_enabled) {
    if (level > level_filter_threshold) {
      ret = false;
    }
  }
  if (level_mask_enabled) {
    if ( !(level_mask & (1 << level)) ) {
      ret = false;
    }
  }
  return ret;
}

/***************************************************************************//**
 * Enable or disable simple log level filter
 ******************************************************************************/
void app_log_filter_threshold_enable(bool enable)
{
  level_filter_enabled = enable;
}

/***************************************************************************//**
 * Enable or disable masking for log levels
 ******************************************************************************/
void app_log_filter_mask_enable(bool enable)
{
  level_mask_enabled = enable;
}

/***************************************************************************//**
 * Set simple filter threshold log level
 ******************************************************************************/
sl_status_t app_log_filter_threshold_set(uint8_t level)
{
  sl_status_t sc = SL_STATUS_OK;
  if (level < APP_LOG_LEVEL_COUNT) {
    level_filter_threshold = level;
  } else {
    sc = SL_STATUS_INVALID_PARAMETER;
  }
  return sc;
}

/***************************************************************************//**
 * Set masks for log levels
 ******************************************************************************/
sl_status_t app_log_filter_mask_set(uint8_t mask)
{
  sl_status_t sc = SL_STATUS_OK;
  if (mask < (1 << (APP_LOG_LEVEL_COUNT))) {
    level_mask = mask;
  } else {
    sc = SL_STATUS_INVALID_PARAMETER;
  }
  return sc;
}

/***************************************************************************//**
 * Get simple filter threshold log level
 ******************************************************************************/
uint8_t app_log_filter_threshold_get(void)
{
  return level_filter_threshold;
}

/***************************************************************************//**
 * Get masks for log levels
 ******************************************************************************/
uint8_t app_log_filter_mask_get(void)
{
  return level_mask;
}

/***************************************************************************//**
 * Sets IO Stream
 ******************************************************************************/
sl_status_t app_log_iostream_set(sl_iostream_t *stream)
{
  sl_status_t ret = SL_STATUS_INVALID_PARAMETER;

  for (uint32_t i = 0; i < sl_iostream_instances_count; i++) {
    if (sl_iostream_instances_info[i]->handle == stream) {
      app_log_iostream = stream;
      ret = SL_STATUS_OK;
      break;
    }
  }
  return ret;
}

/***************************************************************************//**
 * Get the current IO Stream for logging
 ******************************************************************************/
sl_iostream_t * app_log_iostream_get(void)
{
  return app_log_iostream;
}

/******************************************************************************
* Log status string
******************************************************************************/
void _app_log_status_string(sl_status_t sc)
{
  sl_iostream_printf(app_log_iostream, "(");
  sl_iostream_t * default_stream_cfgd = sl_iostream_get_default();
  sl_iostream_set_default(app_log_iostream);
  sl_status_print(sc);
  sl_iostream_set_default(default_stream_cfgd);
  sl_iostream_printf(app_log_iostream, ") ");
}

/***************************************************************************//**
 * Log time
 ******************************************************************************/
SL_WEAK void _app_log_time()
{
  #if APP_LOG_TIME_ENABLE == 1
  #ifdef SL_CATALOG_SLEEPTIMER_PRESENT
  uint32_t time_ms = (uint32_t)(sl_sleeptimer_get_tick_count64()
                                * 1000
                                / sl_sleeptimer_get_timer_frequency());
  time_ms = time_ms; // appease compiler if log is turned off
  app_log_append(APP_LOG_TIME_FORMAT APP_LOG_SEPARATOR,
                 (time_ms / 3600000),
                 (time_ms / 60000) % 60,
                 (time_ms / 1000) % 60,
                 time_ms % 1000);
  #endif // SL_CATALOG_SLEEPTIMER_PRESENT
  #ifdef HOST_TOOLCHAIN
  // Log time for host examples
  #if defined(POSIX) && POSIX == 1
  struct timeval tv;
  struct tm *loc_time;

  if (gettimeofday(&tv, NULL) == 0) {
    loc_time = localtime(&tv.tv_sec);
    if (loc_time != NULL) {
      app_log_append(APP_LOG_TIME_FORMAT APP_LOG_SEPARATOR,
                     loc_time->tm_hour,
                     loc_time->tm_min,
                     loc_time->tm_sec,
                     (uint32_t)(tv.tv_usec / 1000));
    }
  }
  #else
  SYSTEMTIME loc_time;

  GetLocalTime(&loc_time);
  app_log_append(APP_LOG_TIME_FORMAT APP_LOG_SEPARATOR,
                 loc_time.wHour,
                 loc_time.wMinute,
                 loc_time.wSecond,
                 loc_time.wMilliseconds);
  #endif // defined(POSIX) && POSIX == 1
  #endif // HOST_TOOLCHAIN
  #endif // APP_LOG_TIME_ENABLE
}

/***************************************************************************//**
 * Log counter
 ******************************************************************************/
void _app_log_counter()
{
  #if APP_LOG_COUNTER_ENABLE == 1
  app_log_append(APP_LOG_COUNTER_FORMAT
                 APP_LOG_SEPARATOR,
                 counter++);
  #endif // APP_LOG_COUNTER_ENABLE
}

/******************************************************************************
* Application log init
******************************************************************************/
void app_log_init()
{
  // Init filter and mask
  level_filter_enabled = APP_LOG_LEVEL_FILTER_ENABLE;
  level_mask_enabled = APP_LOG_LEVEL_MASK_ENABLE;
  level_filter_threshold = APP_LOG_LEVEL_FILTER_THRESHOLD;
  level_mask = (APP_LOG_LEVEL_MASK_CRITICAL << APP_LOG_LEVEL_CRITICAL)
               | (APP_LOG_LEVEL_MASK_ERROR << APP_LOG_LEVEL_ERROR)
               | (APP_LOG_LEVEL_MASK_WARNING << APP_LOG_LEVEL_WARNING)
               | (APP_LOG_LEVEL_MASK_INFO << APP_LOG_LEVEL_INFO)
               | (APP_LOG_LEVEL_MASK_DEBUG << APP_LOG_LEVEL_DEBUG);
  #if defined(APP_LOG_OVERRIDE_DEFAULT_STREAM) && APP_LOG_OVERRIDE_DEFAULT_STREAM == 1
  char *name = APP_LOG_STREAM_INSTANCE;
  sl_iostream_type_t type = APP_LOG_STREAM_TYPE;

  sl_iostream_t * iostream = NULL;
  sl_iostream_t * iostream_type = NULL;

  for (uint32_t i = 0; i < sl_iostream_instances_count; i++) {
    if (sl_iostream_instances_info[i]->type == type) {
      if (strcmp(sl_iostream_instances_info[i]->name, name) == 0) {
        iostream = sl_iostream_instances_info[i]->handle;
        break;
      }
      if (iostream_type == NULL) {
        iostream_type = sl_iostream_instances_info[i]->handle;
      }
    }
  }

  if (iostream == NULL) {
    // The stream is not found by name
    if (iostream_type != NULL) {
      // Stream found by type
      iostream = iostream_type;
    } else {
      // Not found stream, set to default
      iostream = sl_iostream_get_default();
    }
  }

  app_log_iostream = iostream;
  #else  // APP_LOG_OVERRIDE_DEFAULT_STREAM
  app_log_iostream = sl_iostream_get_default();
  #endif // APP_LOG_OVERRIDE_DEFAULT_STREAM
}

/******************************************************************************
* Missing from SiLabs code
******************************************************************************/
void sl_status_to_string(sl_status_t status, char *buffer, uint32_t buffer_length)
{
  switch (status)
  {
  case SL_STATUS_OK: // ((sl_status_t)0x0000)  ///< No error.
    strncpy(buffer, "OK", buffer_length);
    break;
  case SL_STATUS_FAIL: // ((sl_status_t)0x0001)  ///< Generic error.
    strncpy(buffer, "FAIL", buffer_length);
    break;

// State Errors
  case SL_STATUS_INVALID_STATE: // ((sl_status_t)0x0002)  ///< Generic invalid state error.
    strncpy(buffer, "INVALID_STATE", buffer_length);
    break;
  case SL_STATUS_NOT_READY: // ((sl_status_t)0x0003)  ///< Module is not ready for requested operation.
    strncpy(buffer, "NOT_READY", buffer_length);
    break;
  case SL_STATUS_BUSY: // ((sl_status_t)0x0004)  ///< Module is busy and cannot carry out requested operation.
    strncpy(buffer, "BUSY", buffer_length);
    break;
  case SL_STATUS_IN_PROGRESS: // ((sl_status_t)0x0005)  ///< Operation is in progress and not yet complete (pass or fail).
    strncpy(buffer, "IN_PROGRESS", buffer_length);
    break;
  case SL_STATUS_ABORT: // ((sl_status_t)0x0006)  ///< Operation aborted.
    strncpy(buffer, "ABORT", buffer_length);
    break;
  case SL_STATUS_TIMEOUT: // ((sl_status_t)0x0007)  ///< Operation timed out.
    strncpy(buffer, "TIMEOUT", buffer_length);
    break;
  case SL_STATUS_PERMISSION: // ((sl_status_t)0x0008)  ///< Operation not allowed per permissions.
    strncpy(buffer, "PERMISSION", buffer_length);
    break;
  case SL_STATUS_WOULD_BLOCK: // ((sl_status_t)0x0009)  ///< Non-blocking operation would block.
    strncpy(buffer, "WOULD_BLOCK", buffer_length);
    break;
  case SL_STATUS_IDLE: // ((sl_status_t)0x000A)  ///< Operation/module is Idle, cannot carry requested operation.
    strncpy(buffer, "IDLE", buffer_length);
    break;
  case SL_STATUS_IS_WAITING: // ((sl_status_t)0x000B)  ///< Operation cannot be done while construct is waiting.
    strncpy(buffer, "IS_WAITING", buffer_length);
    break;
  case SL_STATUS_NONE_WAITING: // ((sl_status_t)0x000C)  ///< No task/construct waiting/pending for that action/event.
    strncpy(buffer, "NONE_WAITING", buffer_length);
    break;
  case SL_STATUS_SUSPENDED: // ((sl_status_t)0x000D)  ///< Operation cannot be done while construct is suspended.
    strncpy(buffer, "SUSPENDED", buffer_length);
    break;
  case SL_STATUS_NOT_AVAILABLE: // ((sl_status_t)0x000E)  ///< Feature not available due to software configuration.
    strncpy(buffer, "NOT_AVAILABLE", buffer_length);
    break;
  case SL_STATUS_NOT_SUPPORTED: // ((sl_status_t)0x000F)  ///< Feature not supported.
    strncpy(buffer, "NOT_SUPPORTED", buffer_length);
    break;
  case SL_STATUS_INITIALIZATION: // ((sl_status_t)0x0010)  ///< Initialization failed.
    strncpy(buffer, "INITIALIZATION", buffer_length);
    break;
  case SL_STATUS_NOT_INITIALIZED: // ((sl_status_t)0x0011)  ///< Module has not been initialized.
    strncpy(buffer, "NOT_INITIALIZED", buffer_length);
    break;
  case SL_STATUS_ALREADY_INITIALIZED: // ((sl_status_t)0x0012)  ///< Module has already been initialized.
    strncpy(buffer, "ALREADY_INITIALIZED", buffer_length);
    break;
  case SL_STATUS_DELETED: // ((sl_status_t)0x0013)  ///< Object/construct has been deleted.
    strncpy(buffer, "DELETED", buffer_length);
    break;
  case SL_STATUS_ISR: // ((sl_status_t)0x0014)  ///< Illegal call from ISR.
    strncpy(buffer, "ISR", buffer_length);
    break;
  case SL_STATUS_NETWORK_UP: // ((sl_status_t)0x0015)  ///< Illegal call because network is up.
    strncpy(buffer, "NETWORK_UP", buffer_length);
    break;
  case SL_STATUS_NETWORK_DOWN: // ((sl_status_t)0x0016)  ///< Illegal call because network is down.
    strncpy(buffer, "NETWORK_DOWN", buffer_length);
    break;
  case SL_STATUS_NOT_JOINED: // ((sl_status_t)0x0017)  ///< Failure due to not being joined in a network.
    strncpy(buffer, "NOT_JOINED", buffer_length);
    break;
  case SL_STATUS_NO_BEACONS: // ((sl_status_t)0x0018)  ///< Invalid operation as there are no Guardeners.
    strncpy(buffer, "NO_BEACONS", buffer_length);
    break;

// Allocation/ownership Errors
  case SL_STATUS_ALLOCATION_FAILED: // ((sl_status_t)0x0019)  ///< Generic allocation error.
    strncpy(buffer, "ALLOCATION_FAILED", buffer_length);
    break;
  case SL_STATUS_NO_MORE_RESOURCE: // ((sl_status_t)0x001A)  ///< No more resource available to perform the operation.
    strncpy(buffer, "NO_MORE_RESOURCE", buffer_length);
    break;
  case SL_STATUS_EMPTY: // ((sl_status_t)0x001B)  ///< Item/list/queue is empty.
    strncpy(buffer, "EMPTY", buffer_length);
    break;
  case SL_STATUS_FULL: // ((sl_status_t)0x001C)  ///< Item/list/queue is full.
    strncpy(buffer, "FULL", buffer_length);
    break;
  case SL_STATUS_WOULD_OVERFLOW: // ((sl_status_t)0x001D)  ///< Item would overflow.
    strncpy(buffer, "WOULD_OVERFLOW", buffer_length);
    break;
  case SL_STATUS_HAS_OVERFLOWED: // ((sl_status_t)0x001E)  ///< Item/list/queue has been overflowed.
    strncpy(buffer, "HAS_OVERFLOWED", buffer_length);
    break;
  case SL_STATUS_OWNERSHIP: // ((sl_status_t)0x001F)  ///< Generic ownership error.
    strncpy(buffer, "OWNERSHIP", buffer_length);
    break;
  case SL_STATUS_IS_OWNER: // ((sl_status_t)0x0020)  ///< Already/still owning resource.
    strncpy(buffer, "IS_OWNER", buffer_length);
    break;

// Invalid Parameters Errors
  case SL_STATUS_INVALID_PARAMETER: // ((sl_status_t)0x0021)  ///< Generic invalid argument or consequence of invalid argument.
    strncpy(buffer, "INVALID_PARAMETER", buffer_length);
    break;
  case SL_STATUS_NULL_POINTER: // ((sl_status_t)0x0022)  ///< Invalid null pointer received as argument.
    strncpy(buffer, "NULL_POINTER", buffer_length);
    break;
  case SL_STATUS_INVALID_CONFIGURATION: // ((sl_status_t)0x0023)  ///< Invalid configuration provided.
    strncpy(buffer, "INVALID_CONFIGURATION", buffer_length);
    break;
  case SL_STATUS_INVALID_MODE: // ((sl_status_t)0x0024)  ///< Invalid mode.
    strncpy(buffer, "INVALID_MODE", buffer_length);
    break;
  case SL_STATUS_INVALID_HANDLE: // ((sl_status_t)0x0025)  ///< Invalid handle.
    strncpy(buffer, "INVALID_HANDLE", buffer_length);
    break;
  case SL_STATUS_INVALID_TYPE: // ((sl_status_t)0x0026)  ///< Invalid type for operation.
    strncpy(buffer, "INVALID_TYPE", buffer_length);
    break;
  case SL_STATUS_INVALID_INDEX: // ((sl_status_t)0x0027)  ///< Invalid index.
    strncpy(buffer, "INVALID_INDEX", buffer_length);
    break;
  case SL_STATUS_INVALID_RANGE: // ((sl_status_t)0x0028)  ///< Invalid range.
    strncpy(buffer, "INVALID_RANGE", buffer_length);
    break;
  case SL_STATUS_INVALID_KEY: // ((sl_status_t)0x0029)  ///< Invalid key.
    strncpy(buffer, "INVALID_KEY", buffer_length);
    break;
  case SL_STATUS_INVALID_CREDENTIALS: // ((sl_status_t)0x002A)  ///< Invalid credentials.
    strncpy(buffer, "INVALID_CREDENTIALS", buffer_length);
    break;
  case SL_STATUS_INVALID_COUNT: // ((sl_status_t)0x002B)  ///< Invalid count.
    strncpy(buffer, "INVALID_COUNT", buffer_length);
    break;
  case SL_STATUS_INVALID_SIGNATURE: // ((sl_status_t)0x002C)  ///< Invalid signature / verification failed.
    strncpy(buffer, "INVALID_SIGNATURE", buffer_length);
    break;
  case SL_STATUS_NOT_FOUND: // ((sl_status_t)0x002D)  ///< Item could not be found.
    strncpy(buffer, "NOT_FOUND", buffer_length);
    break;
  case SL_STATUS_ALREADY_EXISTS: // ((sl_status_t)0x002E)  ///< Item already exists.
    strncpy(buffer, "ALREADY_EXISTS", buffer_length);
    break;

// IO/Communication Errors
  case SL_STATUS_IO: // ((sl_status_t)0x002F)  ///< Generic I/O failure.
    strncpy(buffer, "IO", buffer_length);
    break;
  case SL_STATUS_IO_TIMEOUT: // ((sl_status_t)0x0030)  ///< I/O failure due to timeout.
    strncpy(buffer, "IO_TIMEOUT", buffer_length);
    break;
  case SL_STATUS_TRANSMIT: // ((sl_status_t)0x0031)  ///< Generic transmission error.
    strncpy(buffer, "TRANSMIT", buffer_length);
    break;
  case SL_STATUS_TRANSMIT_UNDERFLOW: // ((sl_status_t)0x0032)  ///< Transmit underflowed.
    strncpy(buffer, "TRANSMIT_UNDERFLOW", buffer_length);
    break;
  case SL_STATUS_TRANSMIT_INCOMPLETE: // ((sl_status_t)0x0033)  ///< Transmit is incomplete.
    strncpy(buffer, "TRANSMIT_INCOMPLETE", buffer_length);
    break;
  case SL_STATUS_TRANSMIT_BUSY: // ((sl_status_t)0x0034)  ///< Transmit is busy.
    strncpy(buffer, "TRANSMIT_BUSY", buffer_length);
    break;
  case SL_STATUS_RECEIVE: // ((sl_status_t)0x0035)  ///< Generic reception error.
    strncpy(buffer, "RECEIVE", buffer_length);
    break;
  case SL_STATUS_OBJECT_READ: // ((sl_status_t)0x0036)  ///< Failed to read on/via given object.
    strncpy(buffer, "OBJECT_READ", buffer_length);
    break;
  case SL_STATUS_OBJECT_WRITE: // ((sl_status_t)0x0037)  ///< Failed to write on/via given object.
    strncpy(buffer, "OBJECT_WRITE", buffer_length);
    break;
  case SL_STATUS_MESSAGE_TOO_LONG: // ((sl_status_t)0x0038)  ///< Message is too long.
    strncpy(buffer, "MESSAGE_TOO_LONG", buffer_length);
    break;

// EEPROM/Flash Errors
  case SL_STATUS_EEPROM_MFG_VERSION_MISMATCH: // ((sl_status_t)0x0039)  ///<
    strncpy(buffer, "EEPROM_MFG_VERSION_MISMATCH", buffer_length);
    break;
  case SL_STATUS_EEPROM_STACK_VERSION_MISMATCH: // ((sl_status_t)0x003A)  ///<
    strncpy(buffer, "EEPROM_STACK_VERSION_MISMATCH", buffer_length);
    break;
  case SL_STATUS_FLASH_WRITE_INHIBITED: // ((sl_status_t)0x003B)  ///< Flash write is inhibited.
    strncpy(buffer, "FLASH_WRITE_INHIBITED", buffer_length);
    break;
  case SL_STATUS_FLASH_VERIFY_FAILED: // ((sl_status_t)0x003C)  ///< Flash verification failed.
    strncpy(buffer, "FLASH_VERIFY_FAILED", buffer_length);
    break;
  case SL_STATUS_FLASH_PROGRAM_FAILED: // ((sl_status_t)0x003D)  ///< Flash programming failed.
    strncpy(buffer, "FLASH_PROGRAM_FAILED", buffer_length);
    break;
  case SL_STATUS_FLASH_ERASE_FAILED: // ((sl_status_t)0x003E)  ///< Flash erase failed.
    strncpy(buffer, "FLASH_ERASE_FAILED", buffer_length);
    break;

// MAC Errors
  case SL_STATUS_MAC_NO_DATA: // ((sl_status_t)0x003F)  ///<
    strncpy(buffer, "MAC_NO_DATA", buffer_length);
    break;
  case SL_STATUS_MAC_NO_ACK_RECEIVED: // ((sl_status_t)0x0040)  ///<
    strncpy(buffer, "MAC_NO_ACK_RECEIVED", buffer_length);
    break;
  case SL_STATUS_MAC_INDIRECT_TIMEOUT: // ((sl_status_t)0x0041)  ///<
    strncpy(buffer, "MAC_INDIRECT_TIMEOUT", buffer_length);
    break;
  case SL_STATUS_MAC_UNKNOWN_HEADER_TYPE: // ((sl_status_t)0x0042)  ///<
    strncpy(buffer, "MAC_UNKNOWN_HEADER_TYPE", buffer_length);
    break;
  case SL_STATUS_MAC_ACK_HEADER_TYPE: // ((sl_status_t)0x0043)  ///<
    strncpy(buffer, "MAC_ACK_HEADER_TYPE", buffer_length);
    break;
  case SL_STATUS_MAC_COMMAND_TRANSMIT_FAILURE: // ((sl_status_t)0x0044)  ///<
    strncpy(buffer, "MAC_COMMAND_TRANSMIT_FAILURE", buffer_length);
    break;

// CLI_STORAGE Errors
  case SL_STATUS_CLI_STORAGE_NVM_OPEN_ERROR: // ((sl_status_t)0x0045)  ///< Error in open NVM
    strncpy(buffer, "CLI_STORAGE_NVM_OPEN_ERROR", buffer_length);
    break;

// Security status codes
  case SL_STATUS_SECURITY_IMAGE_CHECKSUM_ERROR: // sl_status_t)0x0046)  ///< Image checksum is not valid.
    strncpy(buffer, "SECURITY_IMAGE_CHECKSUM_ERROR", buffer_length);
    break;
  case SL_STATUS_SECURITY_DECRYPT_ERROR: // ((sl_status_t)0x0047)  ///< Decryption failed
    strncpy(buffer, "SECURITY_DECRYPT_ERROR", buffer_length);
    break;

// Command status codes
  case SL_STATUS_COMMAND_IS_INVALID: // ((sl_status_t)0x0048)  ///< Command was not recognized
    strncpy(buffer, "COMMAND_IS_INVALID", buffer_length);
    break;
  case SL_STATUS_COMMAND_TOO_LONG: // ((sl_status_t)0x0049)  ///< Command or parameter maximum length exceeded
    strncpy(buffer, "COMMAND_TOO_LONG", buffer_length);
    break;
  case SL_STATUS_COMMAND_INCOMPLETE: // ((sl_status_t)0x004A)  ///< Data received does not form a complete command
    strncpy(buffer, "COMMAND_INCOMPLETE", buffer_length);
    break;

// Misc Errors
  case SL_STATUS_BUS_ERROR: // ((sl_status_t)0x004B)  ///< Bus error, e.g. invalid DMA address
    strncpy(buffer, "BUS_ERROR", buffer_length);
    break;

// Unified MAC Errors
  case SL_STATUS_CCA_FAILURE: // ((sl_status_t)0x004C)  ///<
    strncpy(buffer, "CCA_FAILURE", buffer_length);
    break;

// Scan errors
  case SL_STATUS_MAC_SCANNING: // ((sl_status_t)0x004D)  ///<
    strncpy(buffer, "MAC_SCANNING", buffer_length);
    break;
  case SL_STATUS_MAC_INCORRECT_SCAN_TYPE: // ((sl_status_t)0x004E)  ///<
    strncpy(buffer, "MAC_INCORRECT_SCAN_TYPE", buffer_length);
    break;
  case SL_STATUS_INVALID_CHANNEL_MASK: // ((sl_status_t)0x004F)  ///<
    strncpy(buffer, "INVALID_CHANNEL_MASK", buffer_length);
    break;
  case SL_STATUS_BAD_SCAN_DURATION: // ((sl_status_t)0x0050)  ///<
    strncpy(buffer, "BAD_SCAN_DURATION", buffer_length);
    break;

// Bluetooth status codes
  case SL_STATUS_BT_OUT_OF_BONDS: // ((sl_status_t)0x0402)        ///< Bonding procedure can't be started because device has no space left for bond.
    strncpy(buffer, "BT_OUT_OF_BONDS", buffer_length);
    break;
  case SL_STATUS_BT_UNSPECIFIED: // ((sl_status_t)0x0403)        ///< Unspecified error
    strncpy(buffer, "BT_UNSPECIFIED", buffer_length);
    break;
  case SL_STATUS_BT_HARDWARE: // ((sl_status_t)0x0404)        ///< Hardware failure
    strncpy(buffer, "BT_HARDWARE", buffer_length);
    break;
  case SL_STATUS_BT_NO_BONDING: // ((sl_status_t)0x0406)        ///< The bonding does not exist.
    strncpy(buffer, "BT_NO_BONDING", buffer_length);
    break;
  case SL_STATUS_BT_CRYPTO: // ((sl_status_t)0x0407)        ///< Error using crypto functions
    strncpy(buffer, "BT_CRYPTO", buffer_length);
    break;
  case SL_STATUS_BT_DATA_CORRUPTED: // ((sl_status_t)0x0408)        ///< Data was corrupted.
    strncpy(buffer, "BT_DATA_CORRUPTED", buffer_length);
    break;
  case SL_STATUS_BT_INVALID_SYNC_HANDLE: // ((sl_status_t)0x040A)        ///< Invalid periodic advertising sync handle
    strncpy(buffer, "BT_INVALID_SYNC_HANDLE", buffer_length);
    break;
  case SL_STATUS_BT_INVALID_MODULE_ACTION: // ((sl_status_t)0x040B)        ///< Bluetooth cannot be used on this hardware
    strncpy(buffer, "BT_INVALID_MODULE_ACTION", buffer_length);
    break;
  case SL_STATUS_BT_RADIO: // ((sl_status_t)0x040C)        ///< Error received from radio
    strncpy(buffer, "BT_RADIO", buffer_length);
    break;
  case SL_STATUS_BT_L2CAP_REMOTE_DISCONNECTED: // ((sl_status_t)0x040D)        ///< Returned when remote disconnects the connection-oriented channel by sending disconnection request.
    strncpy(buffer, "BT_L2CAP_REMOTE_DISCONNECTED", buffer_length);
    break;
  case SL_STATUS_BT_L2CAP_LOCAL_DISCONNECTED: // ((sl_status_t)0x040E)        ///< Returned when local host disconnect the connection-oriented channel by sending disconnection request.
    strncpy(buffer, "BT_L2CAP_LOCAL_DISCONNECTED", buffer_length);
    break;
  case SL_STATUS_BT_L2CAP_CID_NOT_EXIST: // ((sl_status_t)0x040F)        ///< Returned when local host did not find a connection-oriented channel with given destination CID.
    strncpy(buffer, "BT_L2CAP_CID_NOT_EXIST", buffer_length);
    break;
  case SL_STATUS_BT_L2CAP_LE_DISCONNECTED: // ((sl_status_t)0x0410)        ///< Returned when connection-oriented channel disconnected due to LE connection is dropped.
    strncpy(buffer, "BT_L2CAP_LE_DISCONNECTED", buffer_length);
    break;
  case SL_STATUS_BT_L2CAP_FLOW_CONTROL_VIOLATED: // ((sl_status_t)0x0412)        ///< Returned when connection-oriented channel disconnected due to remote end send data even without credit.
    strncpy(buffer, "BT_L2CAP_FLOW_CONTROL_VIOLATED", buffer_length);
    break;
  case SL_STATUS_BT_L2CAP_FLOW_CONTROL_CREDIT_OVERFLOWED: // ((sl_status_t)0x0413)        ///< Returned when connection-oriented channel disconnected due to remote end send flow control credits exceed 65535.
    strncpy(buffer, "BT_L2CAP_FLOW_CONTROL_CREDIT_OVERFLOWED", buffer_length);
    break;
  case SL_STATUS_BT_L2CAP_NO_FLOW_CONTROL_CREDIT: // ((sl_status_t)0x0414)        ///< Returned when connection-oriented channel has run out of flow control credit and local application still trying to send data.
    strncpy(buffer, "BT_L2CAP_NO_FLOW_CONTROL_CREDIT", buffer_length);
    break;
  case SL_STATUS_BT_L2CAP_CONNECTION_REQUEST_TIMEOUT: // ((sl_status_t)0x0415)        ///< Returned when connection-oriented channel has not received connection response message within maximum timeout.
    strncpy(buffer, "BT_L2CAP_CONNECTION_REQUEST_TIMEOUT", buffer_length);
    break;
  case SL_STATUS_BT_L2CAP_INVALID_CID: // ((sl_status_t)0x0416)        ///< Returned when local host received a connection-oriented channel connection response with an invalid destination CID.
    strncpy(buffer, "BT_L2CAP_INVALID_CID", buffer_length);
    break;
  case SL_STATUS_BT_L2CAP_WRONG_STATE: // ((sl_status_t)0x0417)        ///< Returned when local host application tries to send a command which is not suitable for L2CAP channel's current state.
    strncpy(buffer, "BT_L2CAP_WRONG_STATE", buffer_length);
    break;
  case SL_STATUS_BT_PS_STORE_FULL: // ((sl_status_t)0x041B)        ///< Flash reserved for PS store is full
    strncpy(buffer, "BT_PS_STORE_FULL", buffer_length);
    break;
  case SL_STATUS_BT_PS_KEY_NOT_FOUND: // ((sl_status_t)0x041C)        ///< PS key not found
    strncpy(buffer, "BT_PS_KEY_NOT_FOUND", buffer_length);
    break;
  case SL_STATUS_BT_APPLICATION_MISMATCHED_OR_INSUFFICIENT_SECURITY: // ((sl_status_t)0x041D)        ///< Mismatched or insufficient security level
    strncpy(buffer, "BT_APPLICATION_MISMATCHED_OR_INSUFFICIENT_SECURITY", buffer_length);
    break;
  case SL_STATUS_BT_APPLICATION_ENCRYPTION_DECRYPTION_ERROR: // ((sl_status_t)0x041E)        ///< Encrypion/decryption operation failed.
    strncpy(buffer, "BT_APPLICATION_ENCRYPTION_DECRYPTION_ERROR", buffer_length);
    break;

// Bluetooth controller status codes
  case SL_STATUS_BT_CTRL_UNKNOWN_CONNECTION_IDENTIFIER: // ((sl_status_t)0x1002)      ///< Connection does not exist, or connection open request was cancelled.
    strncpy(buffer, "BT_CTRL_UNKNOWN_CONNECTION_IDENTIFIER", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_AUTHENTICATION_FAILURE: // ((sl_status_t)0x1005)      ///< Pairing or authentication failed due to incorrect results in the pairing or authentication procedure. This could be due to an incorrect PIN or Link Key
    strncpy(buffer, "BT_CTRL_AUTHENTICATION_FAILURE", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_PIN_OR_KEY_MISSING: // ((sl_status_t)0x1006)      ///< Pairing failed because of missing PIN, or authentication failed because of missing Key
    strncpy(buffer, "BT_CTRL_PIN_OR_KEY_MISSING", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_MEMORY_CAPACITY_EXCEEDED: // ((sl_status_t)0x1007)      ///< Controller is out of memory.
    strncpy(buffer, "BT_CTRL_MEMORY_CAPACITY_EXCEEDED", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_CONNECTION_TIMEOUT: // ((sl_status_t)0x1008)      ///< Link supervision timeout has expired.
    strncpy(buffer, "BT_CTRL_CONNECTION_TIMEOUT", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_CONNECTION_LIMIT_EXCEEDED: // ((sl_status_t)0x1009)      ///< Controller is at limit of connections it can support.
    strncpy(buffer, "BT_CTRL_CONNECTION_LIMIT_EXCEEDED", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_SYNCHRONOUS_CONNECTION_LIMIT_EXCEEDED: // ((sl_status_t)0x100A)     ///< The Synchronous Connection Limit to a Device Exceeded error code indicates that the Controller has reached the limit to the number of synchronous connections that can be achieved to a device.
    strncpy(buffer, "BT_CTRL_SYNCHRONOUS_CONNECTION_LIMIT_EXCEEDED", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_ACL_CONNECTION_ALREADY_EXISTS: // ((sl_status_t)0x100B)     ///< The ACL Connection Already Exists error code indicates that an attempt to create a new ACL Connection to a device when there is already a connection to this device.
    strncpy(buffer, "BT_CTRL_ACL_CONNECTION_ALREADY_EXISTS", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_COMMAND_DISALLOWED: // ((sl_status_t)0x100C)     ///< Command requested cannot be executed because the Controller is in a state where it cannot process this command at this time.
    strncpy(buffer, "BT_CTRL_COMMAND_DISALLOWED", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_CONNECTION_REJECTED_DUE_TO_LIMITED_RESOURCES: // ((sl_status_t)0x100D)     ///< The Connection Rejected Due To Limited Resources error code indicates that an incoming connection was rejected due to limited resources.
    strncpy(buffer, "BT_CTRL_CONNECTION_REJECTED_DUE_TO_LIMITED_RESOURCES", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_CONNECTION_REJECTED_DUE_TO_SECURITY_REASONS: // ((sl_status_t)0x100E)     ///< The Connection Rejected Due To Security Reasons error code indicates that a connection was rejected due to security requirements not being fulfilled, like authentication or pairing.
    strncpy(buffer, "BT_CTRL_CONNECTION_REJECTED_DUE_TO_SECURITY_REASONS", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_CONNECTION_REJECTED_DUE_TO_UNACCEPTABLE_BD_ADDR: // ((sl_status_t)0x100F)     ///< The Connection was rejected because this device does not accept the BD_ADDR. This may be because the device will only accept connections from specific BD_ADDRs.
    strncpy(buffer, "BT_CTRL_CONNECTION_REJECTED_DUE_TO_UNACCEPTABLE_BD_ADDR", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_CONNECTION_ACCEPT_TIMEOUT_EXCEEDED: // ((sl_status_t)0x1010)     ///< The Connection Accept Timeout has been exceeded for this connection attempt.
    strncpy(buffer, "BT_CTRL_CONNECTION_ACCEPT_TIMEOUT_EXCEEDED", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE: // ((sl_status_t)0x1011)     ///< A feature or parameter value in the HCI command is not supported.
    strncpy(buffer, "BT_CTRL_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_INVALID_COMMAND_PARAMETERS: // ((sl_status_t)0x1012)     ///< Command contained invalid parameters.
    strncpy(buffer, "BT_CTRL_INVALID_COMMAND_PARAMETERS", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_REMOTE_USER_TERMINATED: // ((sl_status_t)0x1013)     ///< User on the remote device terminated the connection.
    strncpy(buffer, "BT_CTRL_REMOTE_USER_TERMINATED", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_LOW_RESOURCES: // ((sl_status_t)0x1014)     ///< The remote device terminated the connection because of low resources
    strncpy(buffer, "BT_CTRL_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_LOW_RESOURCES", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_REMOTE_POWERING_OFF: // ((sl_status_t)0x1015)     ///< Remote Device Terminated Connection due to Power Off
    strncpy(buffer, "BT_CTRL_REMOTE_POWERING_OFF", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_CONNECTION_TERMINATED_BY_LOCAL_HOST: // ((sl_status_t)0x1016)     ///< Local device terminated the connection.
    strncpy(buffer, "BT_CTRL_CONNECTION_TERMINATED_BY_LOCAL_HOST", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_REPEATED_ATTEMPTS: // ((sl_status_t)0x1017)     ///< The Controller is disallowing an authentication or pairing procedure because too little time has elapsed since the last authentication or pairing attempt failed.
    strncpy(buffer, "BT_CTRL_REPEATED_ATTEMPTS", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_PAIRING_NOT_ALLOWED: // ((sl_status_t)0x1018)     ///< The device does not allow pairing. This can be for example, when a device only allows pairing during a certain time window after some user input allows pairing
    strncpy(buffer, "BT_CTRL_PAIRING_NOT_ALLOWED", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_UNSUPPORTED_REMOTE_FEATURE: // ((sl_status_t)0x101A)     ///< The remote device does not support the feature associated with the issued command.
    strncpy(buffer, "BT_CTRL_UNSUPPORTED_REMOTE_FEATURE", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_UNSPECIFIED_ERROR: // ((sl_status_t)0x101F)     ///< No other error code specified is appropriate to use.
    strncpy(buffer, "BT_CTRL_UNSPECIFIED_ERROR", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_LL_RESPONSE_TIMEOUT: // ((sl_status_t)0x1022)     ///< Connection terminated due to link-layer procedure timeout.
    strncpy(buffer, "BT_CTRL_LL_RESPONSE_TIMEOUT", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_LL_PROCEDURE_COLLISION: // ((sl_status_t)0x1023)     ///< LL procedure has collided with the same transaction or procedure that is already in progress.
    strncpy(buffer, "BT_CTRL_LL_PROCEDURE_COLLISION", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_ENCRYPTION_MODE_NOT_ACCEPTABLE: // ((sl_status_t)0x1025)     ///< The requested encryption mode is not acceptable at this time.
    strncpy(buffer, "BT_CTRL_ENCRYPTION_MODE_NOT_ACCEPTABLE", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_LINK_KEY_CANNOT_BE_CHANGED: // ((sl_status_t)0x1026)     ///< Link key cannot be changed because a fixed unit key is being used.
    strncpy(buffer, "BT_CTRL_LINK_KEY_CANNOT_BE_CHANGED", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_INSTANT_PASSED: // ((sl_status_t)0x1028)     ///< LMP PDU or LL PDU that includes an instant cannot be performed because the instant when this would have occurred has passed.
    strncpy(buffer, "BT_CTRL_INSTANT_PASSED", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED: // ((sl_status_t)0x1029)     ///< It was not possible to pair as a unit key was requested and it is not supported.
    strncpy(buffer, "BT_CTRL_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_DIFFERENT_TRANSACTION_COLLISION: // ((sl_status_t)0x102A)     ///< LMP transaction was started that collides with an ongoing transaction.
    strncpy(buffer, "BT_CTRL_DIFFERENT_TRANSACTION_COLLISION", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_CHANNEL_ASSESSMENT_NOT_SUPPORTED: // ((sl_status_t)0x102E)     ///< The Controller cannot perform channel assessment because it is not supported.
    strncpy(buffer, "BT_CTRL_CHANNEL_ASSESSMENT_NOT_SUPPORTED", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_INSUFFICIENT_SECURITY: // ((sl_status_t)0x102F)     ///< The HCI command or LMP PDU sent is only possible on an encrypted link.
    strncpy(buffer, "BT_CTRL_INSUFFICIENT_SECURITY", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_PARAMETER_OUT_OF_MANDATORY_RANGE: // ((sl_status_t)0x1030)     ///< A parameter value requested is outside the mandatory range of parameters for the given HCI command or LMP PDU.
    strncpy(buffer, "BT_CTRL_PARAMETER_OUT_OF_MANDATORY_RANGE", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST: // ((sl_status_t)0x1037)     ///< The IO capabilities request or response was rejected because the sending Host does not support Secure Simple Pairing even though the receiving Link Manager does.
    strncpy(buffer, "BT_CTRL_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_HOST_BUSY_PAIRING: // ((sl_status_t)0x1038)     ///< The Host is busy with another pairing operation and unable to support the requested pairing. The receiving device should retry pairing again later.
    strncpy(buffer, "BT_CTRL_HOST_BUSY_PAIRING", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_CONNECTION_REJECTED_DUE_TO_NO_SUITABLE_CHANNEL_FOUND: // ((sl_status_t)0x1039)     ///< The Controller could not calculate an appropriate value for the Channel selection operation.
    strncpy(buffer, "BT_CTRL_CONNECTION_REJECTED_DUE_TO_NO_SUITABLE_CHANNEL_FOUND", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_CONTROLLER_BUSY: // ((sl_status_t)0x103A)     ///< Operation was rejected because the controller is busy and unable to process the request.
    strncpy(buffer, "BT_CTRL_CONTROLLER_BUSY", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_UNACCEPTABLE_CONNECTION_INTERVAL: // ((sl_status_t)0x103B)     ///< Remote device terminated the connection because of an unacceptable connection interval.
    strncpy(buffer, "BT_CTRL_UNACCEPTABLE_CONNECTION_INTERVAL", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_ADVERTISING_TIMEOUT: // ((sl_status_t)0x103C)     ///< Ddvertising for a fixed duration completed or, for directed advertising, that advertising completed without a connection being created.
    strncpy(buffer, "BT_CTRL_ADVERTISING_TIMEOUT", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE: // ((sl_status_t)0x103D)     ///< Connection was terminated because the Message Integrity Check (MIC) failed on a received packet.
    strncpy(buffer, "BT_CTRL_CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_CONNECTION_FAILED_TO_BE_ESTABLISHED: // ((sl_status_t)0x103E)     ///< LL initiated a connection but the connection has failed to be established. Controller did not receive any packets from remote end.
    strncpy(buffer, "BT_CTRL_CONNECTION_FAILED_TO_BE_ESTABLISHED", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_MAC_CONNECTION_FAILED: // ((sl_status_t)0x103F)     ///< The MAC of the 802.11 AMP was requested to connect to a peer, but the connection failed.
    strncpy(buffer, "BT_CTRL_MAC_CONNECTION_FAILED", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_COARSE_CLOCK_ADJUSTMENT_REJECTED_BUT_WILL_TRY_TO_ADJUST_USING_CLOCK_DRAGGING: // ((sl_status_t)0x1040)     ///< The master, at this time, is unable to make a coarse adjustment to the piconet clock, using the supplied parameters. Instead the master will attempt to move the clock using clock dragging.
    strncpy(buffer, "BT_CTRL_COARSE_CLOCK_ADJUSTMENT_REJECTED_BUT_WILL_TRY_TO_ADJUST_USING_CLOCK_DRAGGING", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_UNKNOWN_ADVERTISING_IDENTIFIER: // ((sl_status_t)0x1042)     ///< A command was sent from the Host that should identify an Advertising or Sync handle, but the Advertising or Sync handle does not exist.
    strncpy(buffer, "BT_CTRL_UNKNOWN_ADVERTISING_IDENTIFIER", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_LIMIT_REACHED: // ((sl_status_t)0x1043)     ///< Number of operations requested has been reached and has indicated the completion of the activity (e.g., advertising or scanning).
    strncpy(buffer, "BT_CTRL_LIMIT_REACHED", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_OPERATION_CANCELLED_BY_HOST: // ((sl_status_t)0x1044)     ///< A request to the Controller issued by the Host and still pending was successfully canceled.
    strncpy(buffer, "BT_CTRL_OPERATION_CANCELLED_BY_HOST", buffer_length);
    break;
  case SL_STATUS_BT_CTRL_PACKET_TOO_LONG: // ((sl_status_t)0x1045)     ///< An attempt was made to send or receive a packet that exceeds the maximum allowed packet l
    strncpy(buffer, "BT_CTRL_PACKET_TOO_LONG", buffer_length);
    break;

// Bluetooth attribute status codes
  case SL_STATUS_BT_ATT_INVALID_HANDLE: // ((sl_status_t)0x1101)      ///< The attribute handle given was not valid on this server
    strncpy(buffer, "BT_ATT_INVALID_HANDLE", buffer_length);
    break;
  case SL_STATUS_BT_ATT_READ_NOT_PERMITTED: // ((sl_status_t)0x1102)      ///< The attribute cannot be read
    strncpy(buffer, "BT_ATT_READ_NOT_PERMITTED", buffer_length);
    break;
  case SL_STATUS_BT_ATT_WRITE_NOT_PERMITTED: // ((sl_status_t)0x1103)      ///< The attribute cannot be written
    strncpy(buffer, "BT_ATT_WRITE_NOT_PERMITTED", buffer_length);
    break;
  case SL_STATUS_BT_ATT_INVALID_PDU: // ((sl_status_t)0x1104)      ///< The attribute PDU was invalid
    strncpy(buffer, "BT_ATT_INVALID_PDU", buffer_length);
    break;
  case SL_STATUS_BT_ATT_INSUFFICIENT_AUTHENTICATION: // ((sl_status_t)0x1105)      ///< The attribute requires authentication before it can be read or written.
    strncpy(buffer, "BT_ATT_INSUFFICIENT_AUTHENTICATION", buffer_length);
    break;
  case SL_STATUS_BT_ATT_REQUEST_NOT_SUPPORTED: // ((sl_status_t)0x1106)      ///< Attribute Server does not support the request received from the client.
    strncpy(buffer, "BT_ATT_REQUEST_NOT_SUPPORTED", buffer_length);
    break;
  case SL_STATUS_BT_ATT_INVALID_OFFSET: // ((sl_status_t)0x1107)      ///< Offset specified was past the end of the attribute
    strncpy(buffer, "BT_ATT_INVALID_OFFSET", buffer_length);
    break;
  case SL_STATUS_BT_ATT_INSUFFICIENT_AUTHORIZATION: // ((sl_status_t)0x1108)      ///< The attribute requires authorization before it can be read or written.
    strncpy(buffer, "BT_ATT_INSUFFICIENT_AUTHORIZATION", buffer_length);
    break;
  case SL_STATUS_BT_ATT_PREPARE_QUEUE_FULL: // ((sl_status_t)0x1109)      ///< Too many prepare writes have been queueud
    strncpy(buffer, "BT_ATT_PREPARE_QUEUE_FULL", buffer_length);
    break;
  case SL_STATUS_BT_ATT_ATT_NOT_FOUND: // ((sl_status_t)0x110A)     ///< No attribute found within the given attribute handle range.
    strncpy(buffer, "BT_ATT_ATT_NOT_FOUND", buffer_length);
    break;
  case SL_STATUS_BT_ATT_ATT_NOT_LONG: // ((sl_status_t)0x110B)     ///< The attribute cannot be read or written using the Read Blob Request
    strncpy(buffer, "BT_ATT_ATT_NOT_LONG", buffer_length);
    break;
  case SL_STATUS_BT_ATT_INSUFFICIENT_ENC_KEY_SIZE: // ((sl_status_t)0x110C)     ///< The Encryption Key Size used for encrypting this link is insufficient.
    strncpy(buffer, "BT_ATT_INSUFFICIENT_ENC_KEY_SIZE", buffer_length);
    break;
  case SL_STATUS_BT_ATT_INVALID_ATT_LENGTH: // ((sl_status_t)0x110D)     ///< The attribute value length is invalid for the operation
    strncpy(buffer, "BT_ATT_INVALID_ATT_LENGTH", buffer_length);
    break;
  case SL_STATUS_BT_ATT_UNLIKELY_ERROR: // ((sl_status_t)0x110E)     ///< The attribute request that was requested has encountered an error that was unlikely, and therefore could not be completed as requested.
    strncpy(buffer, "BT_ATT_UNLIKELY_ERROR", buffer_length);
    break;
  case SL_STATUS_BT_ATT_INSUFFICIENT_ENCRYPTION: // ((sl_status_t)0x110F)     ///< The attribute requires encryption before it can be read or written.
    strncpy(buffer, "BT_ATT_INSUFFICIENT_ENCRYPTION", buffer_length);
    break;
  case SL_STATUS_BT_ATT_UNSUPPORTED_GROUP_TYPE: // ((sl_status_t)0x1110)     ///< The attribute type is not a supported grouping attribute as defined by a higher layer specification.
    strncpy(buffer, "BT_ATT_UNSUPPORTED_GROUP_TYPE", buffer_length);
    break;
  case SL_STATUS_BT_ATT_INSUFFICIENT_RESOURCES: // ((sl_status_t)0x1111)     ///< Insufficient Resources to complete the request
    strncpy(buffer, "BT_ATT_INSUFFICIENT_RESOURCES", buffer_length);
    break;
  case SL_STATUS_BT_ATT_OUT_OF_SYNC: // ((sl_status_t)0x1112)     ///< The server requests the client to rediscover the database.
    strncpy(buffer, "BT_ATT_OUT_OF_SYNC", buffer_length);
    break;
  case SL_STATUS_BT_ATT_VALUE_NOT_ALLOWED: // ((sl_status_t)0x1113)     ///< The attribute parameter value was not allowed.
    strncpy(buffer, "BT_ATT_VALUE_NOT_ALLOWED", buffer_length);
    break;
  case SL_STATUS_BT_ATT_APPLICATION: // ((sl_status_t)0x1180)    ///< When this is returned in a BGAPI response, the application tried to read or write the value of a user attribute from the GATT databa
    strncpy(buffer, "BT_ATT_APPLICATION", buffer_length);
    break;
  case SL_STATUS_BT_ATT_WRITE_REQUEST_REJECTED: // ((sl_status_t)0x11FC)    ///< The requested write operation cannot be fulfilled for reasons other than permissions.
    strncpy(buffer, "BT_ATT_WRITE_REQUEST_REJECTED", buffer_length);
    break;
  case SL_STATUS_BT_ATT_CLIENT_CHARACTERISTIC_CONFIGURATION_DESCRIPTOR_IMPROPERLY_CONFIGURED: // ((sl_status_t)0x11FD)    ///< The Client Characteristic Configuration descriptor is not configured according to the requirements of the profile or service.
    strncpy(buffer, "BT_ATT_CLIENT_CHARACTERISTIC_CONFIGURATION_DESCRIPTOR_IMPROPERLY_CONFIGURED", buffer_length);
    break;
  case SL_STATUS_BT_ATT_PROCEDURE_ALREADY_IN_PROGRESS: // ((sl_status_t)0x11FE)    ///< The profile or service request cannot be serviced because an operation that has been previously triggered is still in progress.
    strncpy(buffer, "BT_ATT_PROCEDURE_ALREADY_IN_PROGRESS", buffer_length);
    break;
  case SL_STATUS_BT_ATT_OUT_OF_RANGE: // ((sl_status_t)0x11FF)    ///< The attribute value is out of range as defined by a profile or service specification.
    strncpy(buffer, "BT_ATT_OUT_OF_RANGE", buffer_length);
    break;

// Bluetooth Security Manager Protocol status codes
  case SL_STATUS_BT_SMP_PASSKEY_ENTRY_FAILED: // ((sl_status_t)0x1201)      ///< The user input of passkey failed, for example, the user cancelled the operation
    strncpy(buffer, "BT_SMP_PASSKEY_ENTRY_FAILED", buffer_length);
    break;
  case SL_STATUS_BT_SMP_OOB_NOT_AVAILABLE: // ((sl_status_t)0x1202)      ///< Out of Band data is not available for authentication
    strncpy(buffer, "BT_SMP_OOB_NOT_AVAILABLE", buffer_length);
    break;
  case SL_STATUS_BT_SMP_AUTHENTICATION_REQUIREMENTS: // ((sl_status_t)0x1203)      ///< The pairing procedure cannot be performed as authentication requirements cannot be met due to IO capabilities of one or both devices
    strncpy(buffer, "BT_SMP_AUTHENTICATION_REQUIREMENTS", buffer_length);
    break;
  case SL_STATUS_BT_SMP_CONFIRM_VALUE_FAILED: // ((sl_status_t)0x1204)      ///< The confirm value does not match the calculated compare value
    strncpy(buffer, "BT_SMP_CONFIRM_VALUE_FAILED", buffer_length);
    break;
  case SL_STATUS_BT_SMP_PAIRING_NOT_SUPPORTED: // ((sl_status_t)0x1205)      ///< Pairing is not supported by the device
    strncpy(buffer, "BT_SMP_PAIRING_NOT_SUPPORTED", buffer_length);
    break;
  case SL_STATUS_BT_SMP_ENCRYPTION_KEY_SIZE: // ((sl_status_t)0x1206)      ///< The resultant encryption key size is insufficient for the security requirements of this device
    strncpy(buffer, "BT_SMP_ENCRYPTION_KEY_SIZE", buffer_length);
    break;
  case SL_STATUS_BT_SMP_COMMAND_NOT_SUPPORTED: // ((sl_status_t)0x1207)      ///< The SMP command received is not supported on this device
    strncpy(buffer, "BT_SMP_COMMAND_NOT_SUPPORTED", buffer_length);
    break;
  case SL_STATUS_BT_SMP_UNSPECIFIED_REASON: // ((sl_status_t)0x1208)      ///< Pairing failed due to an unspecified reason
    strncpy(buffer, "BT_SMP_UNSPECIFIED_REASON", buffer_length);
    break;
  case SL_STATUS_BT_SMP_REPEATED_ATTEMPTS: // ((sl_status_t)0x1209)      ///< Pairing or authentication procedure is disallowed because too little time has elapsed since last pairing request or security request
    strncpy(buffer, "BT_SMP_REPEATED_ATTEMPTS", buffer_length);
    break;
  case SL_STATUS_BT_SMP_INVALID_PARAMETERS: // ((sl_status_t)0x120A)     ///< The Invalid Parameters error code indicates: the command length is invalid or a parameter is outside of the specified range.
    strncpy(buffer, "BT_SMP_INVALID_PARAMETERS", buffer_length);
    break;
  case SL_STATUS_BT_SMP_DHKEY_CHECK_FAILED: // ((sl_status_t)0x120B)     ///< Indicates to the remote device that the DHKey Check value received doesn't match the one calculated by the local device.
    strncpy(buffer, "BT_SMP_DHKEY_CHECK_FAILED", buffer_length);
    break;
  case SL_STATUS_BT_SMP_NUMERIC_COMPARISON_FAILED: // ((sl_status_t)0x120C)     ///< Indicates that the confirm values in the numeric comparison protocol do not match.
    strncpy(buffer, "BT_SMP_NUMERIC_COMPARISON_FAILED", buffer_length);
    break;
  case SL_STATUS_BT_SMP_BREDR_PAIRING_IN_PROGRESS: // ((sl_status_t)0x120D)     ///< Indicates that the pairing over the LE transport failed due to a Pairing Request sent over the BR/EDR transport in process.
    strncpy(buffer, "BT_SMP_BREDR_PAIRING_IN_PROGRESS", buffer_length);
    break;
  case SL_STATUS_BT_SMP_CROSS_TRANSPORT_KEY_DERIVATION_GENERATION_NOT_ALLOWED: // ((sl_status_t)0x120E)     ///< Indicates that the BR/EDR Link Key generated on the BR/EDR transport cannot be used to derive and distribute keys for the LE transport.
    strncpy(buffer, "BT_SMP_CROSS_TRANSPORT_KEY_DERIVATION_GENERATION_NOT_ALLOWED", buffer_length);
    break;
  case SL_STATUS_BT_SMP_KEY_REJECTED: // ((sl_status_t)0x120F)     ///< Indicates that the device chose not to accept a distributed key.
    strncpy(buffer, "BT_SMP_KEY_REJECTED", buffer_length);
    break;

// Bluetooth Mesh status codes
  case SL_STATUS_BT_MESH_ALREADY_EXISTS: // ((sl_status_t)0x0501)      ///< Returned when trying to add a key or some other unique resource with an ID which already exists
    strncpy(buffer, "BT_MESH_ALREADY_EXISTS", buffer_length);
    break;
  case SL_STATUS_BT_MESH_DOES_NOT_EXIST: // ((sl_status_t)0x0502)      ///< Returned when trying to manipulate a key or some other resource with an ID which does not exist
    strncpy(buffer, "BT_MESH_DOES_NOT_EXIST", buffer_length);
    break;
  case SL_STATUS_BT_MESH_LIMIT_REACHED: // ((sl_status_t)0x0503)      ///< Returned when an operation cannot be executed because a pre-configured limit for keys, key bindings, elements, models, virtual addresses, provisioned devices, or provisioning sessions is reached
    strncpy(buffer, "BT_MESH_LIMIT_REACHED", buffer_length);
    break;
  case SL_STATUS_BT_MESH_INVALID_ADDRESS: // ((sl_status_t)0x0504)      ///< Returned when trying to use a reserved address or add a "pre-provisioned" device using an address already used by some other device
    strncpy(buffer, "BT_MESH_INVALID_ADDRESS", buffer_length);
    break;
  case SL_STATUS_BT_MESH_MALFORMED_DATA: // ((sl_status_t)0x0505)      ///< In a BGAPI response, the user supplied malformed data; in a BGAPI event, the remote end responded with malformed or unrecognized data
    strncpy(buffer, "BT_MESH_MALFORMED_DATA", buffer_length);
    break;
  case SL_STATUS_BT_MESH_ALREADY_INITIALIZED: // ((sl_status_t)0x0506)      ///< An attempt was made to initialize a subsystem that was already initialized.
    strncpy(buffer, "BT_MESH_ALREADY_INITIALIZED", buffer_length);
    break;
  case SL_STATUS_BT_MESH_NOT_INITIALIZED: // ((sl_status_t)0x0507)      ///< An attempt was made to use a subsystem that wasn't initialized yet. Call the subsystem's init function first.
    strncpy(buffer, "BT_MESH_NOT_INITIALIZED", buffer_length);
    break;
  case SL_STATUS_BT_MESH_NO_FRIEND_OFFER: // ((sl_status_t)0x0508)      ///< Returned when trying to establish a friendship as a Low Power Node, but no acceptable friend offer message was received.
    strncpy(buffer, "BT_MESH_NO_FRIEND_OFFER", buffer_length);
    break;
  case SL_STATUS_BT_MESH_PROV_LINK_CLOSED: // ((sl_status_t)0x0509)      ///< Provisioning link was unexpectedly closed before provisioning was complete.
    strncpy(buffer, "BT_MESH_PROV_LINK_CLOSED", buffer_length);
    break;
  case SL_STATUS_BT_MESH_PROV_INVALID_PDU: // ((sl_status_t)0x050A)     ///< An unrecognized provisioning PDU was received.
    strncpy(buffer, "BT_MESH_PROV_INVALID_PDU", buffer_length);
    break;
  case SL_STATUS_BT_MESH_PROV_INVALID_PDU_FORMAT: // ((sl_status_t)0x050B)     ///< A provisioning PDU with wrong length or containing field values that are out of bounds was received.
    strncpy(buffer, "BT_MESH_PROV_INVALID_PDU_FORMAT", buffer_length);
    break;
  case SL_STATUS_BT_MESH_PROV_UNEXPECTED_PDU: // ((sl_status_t)0x050C)     ///< An unexpected (out of sequence) provisioning PDU was received.
    strncpy(buffer, "BT_MESH_PROV_UNEXPECTED_PDU", buffer_length);
    break;
  case SL_STATUS_BT_MESH_PROV_CONFIRMATION_FAILED: // ((sl_status_t)0x050D)     ///< The computed confirmation value did not match the expected value.
    strncpy(buffer, "BT_MESH_PROV_CONFIRMATION_FAILED", buffer_length);
    break;
  case SL_STATUS_BT_MESH_PROV_OUT_OF_RESOURCES: // ((sl_status_t)0x050E)     ///< Provisioning could not be continued due to insufficient resources.
    strncpy(buffer, "BT_MESH_PROV_OUT_OF_RESOURCES", buffer_length);
    break;
  case SL_STATUS_BT_MESH_PROV_DECRYPTION_FAILED: // ((sl_status_t)0x050F)     ///< The provisioning data block could not be decrypted.
    strncpy(buffer, "BT_MESH_PROV_DECRYPTION_FAILED", buffer_length);
    break;
  case SL_STATUS_BT_MESH_PROV_UNEXPECTED_ERROR: // ((sl_status_t)0x0510)     ///< An unexpected error happened during provisioning.
    strncpy(buffer, "BT_MESH_PROV_UNEXPECTED_ERROR", buffer_length);
    break;
  case SL_STATUS_BT_MESH_PROV_CANNOT_ASSIGN_ADDR: // ((sl_status_t)0x0511)     ///< Device could not assign unicast addresses to all of its elements.
    strncpy(buffer, "BT_MESH_PROV_CANNOT_ASSIGN_ADDR", buffer_length);
    break;
  case SL_STATUS_BT_MESH_ADDRESS_TEMPORARILY_UNAVAILABLE: // ((sl_status_t)0x0512)     ///< Returned when trying to reuse an address of a previously deleted device before an IV Index Update has been executed.
    strncpy(buffer, "BT_MESH_ADDRESS_TEMPORARILY_UNAVAILABLE", buffer_length);
    break;
  case SL_STATUS_BT_MESH_ADDRESS_ALREADY_USED: // ((sl_status_t)0x0513)     ///< Returned when trying to assign an address that is used by one of the devices in the Device Database, or by the Provisioner itself.
    strncpy(buffer, "BT_MESH_ADDRESS_ALREADY_USED", buffer_length);
    break;
  case SL_STATUS_BT_MESH_PUBLISH_NOT_CONFIGURED: // ((sl_status_t)0x0514)     ///< Application key or publish address are not set
    strncpy(buffer, "BT_MESH_PUBLISH_NOT_CONFIGURED", buffer_length);
    break;
  case SL_STATUS_BT_MESH_APP_KEY_NOT_BOUND: // ((sl_status_t)0x0515)     ///< Application key is not bound to a model
    strncpy(buffer, "BT_MESH_APP_KEY_NOT_BOUND", buffer_length);
    break;
// Bluetooth Mesh foundation status codes
  case SL_STATUS_BT_MESH_FOUNDATION_INVALID_ADDRESS: // ((sl_status_t)0x1301)      ///< Returned when address in request was not valid
    strncpy(buffer, "BT_MESH_FOUNDATION_INVALID_ADDRESS", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_INVALID_MODEL: // ((sl_status_t)0x1302)      ///< Returned when model identified is not found for a given element
    strncpy(buffer, "BT_MESH_FOUNDATION_INVALID_MODEL", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_INVALID_APP_KEY: // ((sl_status_t)0x1303)      ///< Returned when the key identified by AppKeyIndex is not stored in the node
    strncpy(buffer, "BT_MESH_FOUNDATION_INVALID_APP_KEY", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_INVALID_NET_KEY: // ((sl_status_t)0x1304)      ///< Returned when the key identified by NetKeyIndex is not stored in the node
    strncpy(buffer, "BT_MESH_FOUNDATION_INVALID_NET_KEY", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_INSUFFICIENT_RESOURCES: // ((sl_status_t)0x1305)      ///< Returned when The node cannot serve the request due to insufficient resources
    strncpy(buffer, "BT_MESH_FOUNDATION_INSUFFICIENT_RESOURCES", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_KEY_INDEX_EXISTS: // ((sl_status_t)0x1306)      ///< Returned when the key identified is already stored in the node and the new NetKey value is different
    strncpy(buffer, "BT_MESH_FOUNDATION_KEY_INDEX_EXISTS", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_INVALID_PUBLISH_PARAMS: // ((sl_status_t)0x1307)      ///< Returned when the model does not support the publish mechanism
    strncpy(buffer, "BT_MESH_FOUNDATION_INVALID_PUBLISH_PARAMS", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_NOT_SUBSCRIBE_MODEL: // ((sl_status_t)0x1308)      ///< Returned when  the model does not support the subscribe mechanism
    strncpy(buffer, "BT_MESH_FOUNDATION_NOT_SUBSCRIBE_MODEL", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_STORAGE_FAILURE: // ((sl_status_t)0x1309)      ///< Returned when storing of the requested parameters failed
    strncpy(buffer, "BT_MESH_FOUNDATION_STORAGE_FAILURE", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_NOT_SUPPORTED: // ((sl_status_t)0x130A)     ///< Returned when requested setting is not supported
    strncpy(buffer, "BT_MESH_FOUNDATION_NOT_SUPPORTED", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_CANNOT_UPDATE: // ((sl_status_t)0x130B)     ///< Returned when the requested update operation cannot be performed due to general constraints
    strncpy(buffer, "BT_MESH_FOUNDATION_CANNOT_UPDATE", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_CANNOT_REMOVE: // ((sl_status_t)0x130C)     ///< Returned when the requested delete operation cannot be performed due to general constraints
    strncpy(buffer, "BT_MESH_FOUNDATION_CANNOT_REMOVE", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_CANNOT_BIND: // ((sl_status_t)0x130D)     ///< Returned when the requested bind operation cannot be performed due to general constraints
    strncpy(buffer, "BT_MESH_FOUNDATION_CANNOT_BIND", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_TEMPORARILY_UNABLE: // ((sl_status_t)0x130E)     ///< Returned when The node cannot start advertising with Node Identity or Proxy since the maximum number of parallel advertising is reached
    strncpy(buffer, "BT_MESH_FOUNDATION_TEMPORARILY_UNABLE", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_CANNOT_SET: // ((sl_status_t)0x130F)     ///< Returned when the requested state cannot be set
    strncpy(buffer, "BT_MESH_FOUNDATION_CANNOT_SET", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_UNSPECIFIED: // ((sl_status_t)0x1310)     ///< Returned when an unspecified error took place
    strncpy(buffer, "BT_MESH_FOUNDATION_UNSPECIFIED", buffer_length);
    break;
  case SL_STATUS_BT_MESH_FOUNDATION_INVALID_BINDING: // ((sl_status_t)0x1311)     ///< Returned when the NetKeyIndex and AppKeyIndex combination is not valid for a Config AppKey Update
    strncpy(buffer, "BT_MESH_FOUNDATION_INVALID_BINDING", buffer_length);
    break;
// Wi-Fi Errors
  case SL_STATUS_WIFI_INVALID_KEY: // ((sl_status_t)0x0B01)  ///< Invalid firmware keyset
    strncpy(buffer, "WIFI_INVALID_KEY", buffer_length);
    break;
  case SL_STATUS_WIFI_FIRMWARE_DOWNLOAD_TIMEOUT: // ((sl_status_t)0x0B02)  ///< The firmware download took too long
    strncpy(buffer, "WIFI_FIRMWARE_DOWNLOAD_TIMEOUT", buffer_length);
    break;
  case SL_STATUS_WIFI_UNSUPPORTED_MESSAGE_ID: // ((sl_status_t)0x0B03)  ///< Unknown request ID or wrong interface ID used
    strncpy(buffer, "WIFI_UNSUPPORTED_MESSAGE_ID", buffer_length);
    break;
  case SL_STATUS_WIFI_WARNING: // ((sl_status_t)0x0B04)  ///< The request is successful but some parameters have been ignored
    strncpy(buffer, "WIFI_WARNING", buffer_length);
    break;
  case SL_STATUS_WIFI_NO_PACKET_TO_RECEIVE: // ((sl_status_t)0x0B05)  ///< No Packets waiting to be received
    strncpy(buffer, "WIFI_NO_PACKET_TO_RECEIVE", buffer_length);
    break;
  case SL_STATUS_WIFI_SLEEP_GRANTED: // ((sl_status_t)0x0B08)  ///< The sleep mode is granted
    strncpy(buffer, "WIFI_SLEEP_GRANTED", buffer_length);
    break;
  case SL_STATUS_WIFI_SLEEP_NOT_GRANTED: // ((sl_status_t)0x0B09)  ///< The WFx does not go back to sleep
    strncpy(buffer, "WIFI_SLEEP_NOT_GRANTED", buffer_length);
    break;
  case SL_STATUS_WIFI_SECURE_LINK_MAC_KEY_ERROR: // ((sl_status_t)0x0B10)  ///< The SecureLink MAC key was not found
    strncpy(buffer, "WIFI_SECURE_LINK_MAC_KEY_ERROR", buffer_length);
    break;
  case SL_STATUS_WIFI_SECURE_LINK_MAC_KEY_ALREADY_BURNED: // ((sl_status_t)0x0B11)  ///< The SecureLink MAC key is already installed in OTP
    strncpy(buffer, "WIFI_SECURE_LINK_MAC_KEY_ALREADY_BURNED", buffer_length);
    break;
  case SL_STATUS_WIFI_SECURE_LINK_RAM_MODE_NOT_ALLOWED: // ((sl_status_t)0x0B12)  ///< The SecureLink MAC key cannot be installed in RAM
    strncpy(buffer, "WIFI_SECURE_LINK_RAM_MODE_NOT_ALLOWED", buffer_length);
    break;
  case SL_STATUS_WIFI_SECURE_LINK_FAILED_UNKNOWN_MODE: // ((sl_status_t)0x0B13)  ///< The SecureLink MAC key installation failed
    strncpy(buffer, "WIFI_SECURE_LINK_FAILED_UNKNOWN_MODE", buffer_length);
    break;
  case SL_STATUS_WIFI_SECURE_LINK_EXCHANGE_FAILED: // ((sl_status_t)0x0B14)  ///< SecureLink key (re)negotiation failed
    strncpy(buffer, "WIFI_SECURE_LINK_EXCHANGE_FAILED", buffer_length);
    break;
  case SL_STATUS_WIFI_WRONG_STATE: // ((sl_status_t)0x0B18)  ///< The device is in an inappropriate state to perform the request
    strncpy(buffer, "WIFI_WRONG_STATE", buffer_length);
    break;
  case SL_STATUS_WIFI_CHANNEL_NOT_ALLOWED: // ((sl_status_t)0x0B19)  ///< The request failed due to regulatory limitations
    strncpy(buffer, "WIFI_CHANNEL_NOT_ALLOWED", buffer_length);
    break;
  case SL_STATUS_WIFI_NO_MATCHING_AP: // ((sl_status_t)0x0B1A)  ///< The connection request failed because no suitable AP was found
    strncpy(buffer, "WIFI_NO_MATCHING_AP", buffer_length);
    break;
  case SL_STATUS_WIFI_CONNECTION_ABORTED: // ((sl_status_t)0x0B1B)  ///< The connection request was aborted by host
    strncpy(buffer, "WIFI_CONNECTION_ABORTED", buffer_length);
    break;
  case SL_STATUS_WIFI_CONNECTION_TIMEOUT: // ((sl_status_t)0x0B1C)  ///< The connection request failed because of a timeout
    strncpy(buffer, "WIFI_CONNECTION_TIMEOUT", buffer_length);
    break;
  case SL_STATUS_WIFI_CONNECTION_REJECTED_BY_AP: // ((sl_status_t)0x0B1D)  ///< The connection request failed because the AP rejected the device
    strncpy(buffer, "WIFI_CONNECTION_REJECTED_BY_AP", buffer_length);
    break;
  case SL_STATUS_WIFI_CONNECTION_AUTH_FAILURE: // ((sl_status_t)0x0B1E)  ///< The connection request failed because the WPA handshake did not complete successfully
    strncpy(buffer, "WIFI_CONNECTION_AUTH_FAILURE", buffer_length);
    break;
  case SL_STATUS_WIFI_RETRY_EXCEEDED: // ((sl_status_t)0x0B1F)  ///< The request failed because the retry limit was exceeded
    strncpy(buffer, "WIFI_RETRY_EXCEEDED", buffer_length);
    break;
  case SL_STATUS_WIFI_TX_LIFETIME_EXCEEDED: // ((sl_status_t)0x0B20)  ///< The request failed because the MSDU life time was exceeded
    strncpy(buffer, "WIFI_TX_LIFETIME_EXCEEDED", buffer_length);
    break;

  default:
    snprintf(buffer, buffer_length, "UNKNOWN STATUS: %lu", status);
    break;
  }
}

/******************************************************************************
* Weak implementation of status print
******************************************************************************/
SL_WEAK void sl_status_print(sl_status_t status)
{
  (void) status;
  app_log_append(APP_LOG_UNRESOLVED_STATUS);
}
