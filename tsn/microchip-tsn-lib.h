/* SPDX-License-Identifier: MIT */
/**
 * Microchip CoreTSN API Library
 *
 * Copyright (C) 2025 Microchip Technology Inc. and its subsidiaries
 *
 * Author: Pallela Venkat Karthik <pallela.karthik@microchip.com>
 *
 */

#ifndef _MICROCHIP_TSN_LIB_H_
#define _MICROCHIP_TSN_LIB_H_

#include <stdint.h>
#include <endian.h>
#include <string.h>
#include <linux/types.h>

#define ETHER_ADDR_LEN			6
#define MAX_TSN_CONFIG_SIZE		4096
#define MICROCHIP_TSN_NUM_PRIO_QUEUES_V2 7
#define MICROCHIP_TSN_NUM_PRIO_QUEUES_V3 8
#define MICROCHIP_TSN_NUM_PRIO_QUEUES MICROCHIP_TSN_NUM_PRIO_QUEUES_V3
#define MICROCHIP_TSN_NUM_STREAM_ID_PER_Q_V3 2
#define MCHP_TSN_CDEV_PATH_PREFIX	"/dev/mchpcoretsn"

#define MICROCHIP_TSN_NUM_PSFP_port_STREAM_ID 8
#define MICROCHIP_TSN_NUM_PSFP_port 2

struct microchip_tsn_device {
	__u64 tsn_dev_id;
};

struct gcl_entry {
	__u32 time_interval;
	__u8  gate_state;
};

struct qbv_streamid_conf {
	__u8 mask;
	__u8 da[ETHER_ADDR_LEN];
	__u16 vid;
	__u8 pcp;
	__u8 frer;
};

struct qbv_prioq_streamids_conf {
	struct qbv_streamid_conf streamid[MICROCHIP_TSN_NUM_STREAM_ID_PER_Q_V3];
};

struct qbv_conf {
	__u8 initial_gate_state;
	__u8 priority_enable;
	__u8 gate_enable;
	__u8 priority_queue_enable;
	__u8 priority_queue_prios[MICROCHIP_TSN_NUM_PRIO_QUEUES_V3];
	struct qbv_prioq_streamids_conf priority_queue_prios_que[MICROCHIP_TSN_NUM_PRIO_QUEUES_V3];
	__u8 control_list_length;
	__u64 cycle_time;
	__u64 basetime_sec;
	__u32 basetime_nsec;
	__u8 basetime_adjust;
	__u32 rx_streamid_reset;
	struct gcl_entry gcle[];
};

struct qbu_conf {
	__u8 pre_empt_en;
	__u16 pre_empt_size;
};

struct psfp_stream_conf {
	__u8 mask;
	__u8 da[ETHER_ADDR_LEN];
	__u16 vid;
	__u16 max_sdu_size;
	__u32 psfp_cbs;
	__u32 psfp_ebs;
	__u32 psfp_port_cir;
	__u32 psfp_port_eir;
	__u8 psfp_port_filter_en_dis;
	__u8 psfp_port_fm_en_dis;
	__u8 psfp_port_drop_on_yellow;
	__u8 max_sdu_size_exceed;
	__u8 psfp_cfg_update;
};

struct psfp_port_conf {
	struct psfp_stream_conf port_psfp[MICROCHIP_TSN_NUM_PSFP_port_STREAM_ID];
};

struct qci_conf_v3 {
	struct psfp_port_conf ports[MICROCHIP_TSN_NUM_PSFP_port];
};

struct qci_conf {
	__u8 da_check;
	__u8 sa_check;
	__u8 destination_mac_addr[ETHER_ADDR_LEN];
	__u8 source_mac_addr[ETHER_ADDR_LEN];
};

struct misc_rx_port_id_conf {
	__u8 port_id_rx_check;
	__u16 port_id_rx;
};

struct misc_ptp_tx_prioq_conf {
	__u8 ptp_tx_prioq;
};

struct qav_cbs_q_conf {
	__u8 cbs_q_num;
	__u8 cbs_en;
	__u16 cbs_inc;
	__u16 cbs_dec;
	__u32 cred_min;
	__u32 cred_max;
};

struct qav_conf {
	__u8 num_cbs_queues;
	struct qav_cbs_q_conf cqc[];
};

struct misc_length_deduct_byte_conf {
	__u16 crc_deduct_len;
};

struct pfsp_stream_info_statistics {
	__u32 pfsp_packets_dropped;
	__u32 pfsp_packets_rcvd;
};

struct stream_info_statistics {
	__u32 packets_dropped;
	__u32 packets_sent;
};

struct pfsp_stream_id_statistics {
	struct pfsp_stream_info_statistics pfsp_stream_info[MICROCHIP_TSN_NUM_PSFP_port_STREAM_ID];
};

struct priority_queue_statistics {
	struct stream_info_statistics stream_info[MICROCHIP_TSN_NUM_STREAM_ID_PER_Q_V3];
};

struct tsn_statistics {
	struct priority_queue_statistics queues[MICROCHIP_TSN_NUM_PRIO_QUEUES_V3];
	__u32 rx_port0_prmpt_pkts_drop;
	__u32 rx_port0_prmpt_pkts_rcvd;
	__u32 rx_port0_exp_pkts_drop;
	__u32 rx_port0_exp_pkts_rcvd;
	__u32 rx_port1_prmpt_pkts_drop;
	__u32 rx_port1_prmpt_pkts_rcvd;
	__u32 rx_port1_exp_pkts_drop;
	__u32 rx_port1_exp_pkts_rcvd;
	__u32 tx_port0_exp_pkts;
	__u32 tx_port0_prmpt_pkts;
	__u32 tx_port1_exp_pkts;
	__u32 tx_port1_prmpt_pkts;
	struct pfsp_stream_id_statistics ports[MICROCHIP_TSN_NUM_PSFP_port];
};

struct microchip_tsn_caps {
	__u8 rtl_ver;
	__u8 num_queues;
	__u8 num_streamid_per_q;
};

int microchip_tsn_get_device_list(struct microchip_tsn_device *dev, int max_devices);
int microchip_tsn_device_get_qbv_conf(struct microchip_tsn_device *dev, struct qbv_conf *conf);
int microchip_tsn_device_set_qbv_conf(struct microchip_tsn_device *dev, struct qbv_conf *conf);
int microchip_tsn_device_get_qbu_conf(struct microchip_tsn_device *dev, struct qbu_conf *conf);
int microchip_tsn_device_set_qbu_conf(struct microchip_tsn_device *dev, struct qbu_conf *conf);
int microchip_tsn_device_get_qci_conf(struct microchip_tsn_device *dev, struct qci_conf *conf);
int microchip_tsn_device_set_qci_conf(struct microchip_tsn_device *dev, struct qci_conf *conf);

int microchip_tsn_misc_get_rx_port_id(struct microchip_tsn_device *dev,
				      struct misc_rx_port_id_conf *conf);

int microchip_tsn_misc_set_rx_port_id(struct microchip_tsn_device *dev,
				      struct misc_rx_port_id_conf *conf);

int microchip_tsn_misc_get_tx_ptp_prioq(struct microchip_tsn_device *dev,
					struct misc_ptp_tx_prioq_conf *conf);

int microchip_tsn_misc_set_tx_ptp_prioq(struct microchip_tsn_device *dev,
					struct misc_ptp_tx_prioq_conf *conf);

int microchip_tsn_misc_get_length_deduct_byte(struct microchip_tsn_device *dev,
					      struct misc_length_deduct_byte_conf *conf);

int microchip_tsn_misc_set_length_deduct_byte(struct microchip_tsn_device *dev,
					      struct misc_length_deduct_byte_conf *conf);

int microchip_tsn_device_get_qav_conf(struct microchip_tsn_device *dev,
				      struct qav_conf *conf);

int microchip_tsn_device_set_qav_conf(struct microchip_tsn_device *dev,
				      struct qav_conf *conf);

int microchip_tsn_device_get_caps(struct microchip_tsn_device *dev,
				  struct microchip_tsn_caps *caps);

int microchip_tsn_device_get_qci_conf_v3(struct microchip_tsn_device *dev,
					 struct qci_conf_v3 *conf);

int microchip_tsn_device_set_qci_conf_v3(struct microchip_tsn_device *dev,
					 struct qci_conf_v3 *conf);

int microchip_tsn_device_get_streamid_stats(struct microchip_tsn_device *dev,
					    struct tsn_statistics *stats);

#endif
