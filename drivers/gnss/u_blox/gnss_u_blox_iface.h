/*
 * Copyright (c) 2024 NXP
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _GNSS_U_BLOX_IFACE_H_
#define _GNSS_U_BLOX_IFACE_H_

#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/modem/ubx.h>
#include <zephyr/modem/backend/uart.h>
#include <zephyr/drivers/gpio.h>

#include "gnss_ubx_common.h"

struct u_blox_iface_config {
	const struct device *bus;
	struct gpio_dt_spec reset_gpio;
	uint16_t fix_rate_ms;
	struct {
		uint32_t initial;
		uint32_t desired;
	} baudrate;
};

struct u_blox_iface_data {
	struct gnss_ubx_common_data common_data;
	struct {
		struct modem_pipe *pipe;
		struct modem_backend_uart uart_backend;
		uint8_t receive_buf[1024];
		uint8_t transmit_buf[256];
	} backend;
	struct {
		struct modem_ubx inst;
		uint8_t receive_buf[1024];
	} ubx;
	struct {
		struct modem_ubx_script inst;
		uint8_t response_buf[512];
		uint8_t request_buf[256];
		/*
		 * Serializes every UBX script with interface suspend/resume.  The
		 * lifecycle helpers take this semaphore before detaching the parser,
		 * so a PM transition can never invalidate a pipe while a script uses
		 * it.  It is initialized once at boot and is deliberately retained
		 * across every runtime PM cycle.
		 */
		struct k_sem req_buf_lock;
		struct k_sem lock;
		/* Protected by @ref lock.  False means the parser is detached and
		 * message APIs fail immediately instead of waiting on a released UBX
		 * script semaphore while the GNSS device is suspended.
		 */
		bool active;
	} script;
#if CONFIG_GNSS_SATELLITES
	struct gnss_satellite satellites[CONFIG_GNSS_U_BLOX_SATELLITES_COUNT];
#endif
};

/**
 * @brief Initialize the UBX interface, modem backend, and register unsolicited messages.
 *
 * Must be called before any other APIs can be used.
 *
 * @param[in] dev		GNSS device instance.
 * @param[in] unsol		Unsolicited message array, used to initialize the
 *				backend.
 * @param[in] unsol_size	Number of entries in @a unsol.
 * @param[in] valset_supported	Use UBX-CFG-VALSET when true to configure the baudrate,
 *				otherwise use the legacy UBX-CFG-PRT.
 *
 * @return 0 on success, negative errno on failure.
 */
int u_blox_iface_init(const struct device *dev, const struct modem_ubx_match *unsol,
		      size_t unsol_size, bool valset_supported);

/**
 * @brief Detach the UBX parser and close its UART pipe for runtime suspend.
 *
 * Waits for an in-flight UBX script, then prevents new scripts before the
 * parser is released.  It retains parser, backend, and semaphore objects so
 * @ref u_blox_iface_resume can reuse them.  Repeated suspends are harmless.
 *
 * @param[in] dev	GNSS device instance.
 *
 * @return 0 on success, negative errno on failure.
 */
int u_blox_iface_suspend(const struct device *dev);

/**
 * @brief Reopen and reattach the retained UART/UBX interface after suspend.
 *
 * This only restores transport and parsing.  The GNSS-specific driver must
 * restore its receiver configuration after a successful return.  Repeated
 * resumes while already active are harmless.
 *
 * @param[in] dev	GNSS device instance.
 *
 * @return 0 on success, negative errno on failure.
 */
int u_blox_iface_resume(const struct device *dev);

/**
 * @brief Recover the receiver's configured UART baud rate and reattach it.
 *
 * The caller has already resumed the interface.  This compatibility helper is
 * used when a cold receiver still speaks its devicetree initial baud rate.
 * It is not a general GNSS configuration API.
 */
int u_blox_iface_configure_baudrate(const struct device *dev, bool valset_supported);

/**
 * @brief Send a UBX formatted request and retrieve the response.
 *
 * @param[in] dev		GNSS device instance.
 * @param[in] req		UBX request frame.
 * @param[in] len		Request frame length.
 * @param[out] rsp		Response payload buffer.
 * @param[in] min_rsp_size	Size of the @a rsp buffer.
 *
 * @return 0 on success, negative errno on failure.
 */
int u_blox_iface_msg_get(const struct device *dev, const struct ubx_frame *req,
			 size_t len, void *rsp, size_t min_rsp_size);

/**
 * @brief Send a UBX formatted message
 *
 * @param[in] dev		GNSS device instance.
 * @param[in] req		UBX request frame.
 * @param[in] len		Request frame length.
 * @param[in] wait_for_ack	When set to true, will wait for ACK.
 *
 * @return 0 on success, negative errno on failure.
 */
int u_blox_iface_msg_send(const struct device *dev, const struct ubx_frame *req,
			  size_t len, bool wait_for_ack);

/**
 * @brief Format a payload in a UBX request and send.
 *
 * @param[in] dev		GNSS device instance.
 * @param[in] class_id		UBX message class.
 * @param[in] msg_id		UBX message ID.
 * @param[in] payload		Payload buffer.
 * @param[in] payload_size	Payload size in bytes.
 * @param[in] wait_for_ack	When set to true, will wait for ACK
 *
 * @return 0 on success, negative errno on failure.
 */
int u_blox_iface_msg_payload_send(const struct device *dev, uint8_t class_id, uint8_t msg_id,
				  const uint8_t *payload, size_t payload_size, bool wait_for_ack);

#endif /* _GNSS_U_BLOX_IFACE_H_ */
