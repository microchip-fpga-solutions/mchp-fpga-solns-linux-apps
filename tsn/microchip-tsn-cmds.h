/* SPDX-License-Identifier: MIT */
/**
 * Microchip CoreTSN Configuration Commands
 *
 * Copyright (C) 2025 Microchip Technology Inc. and its subsidiaries
 *
 * Author: Pallela Venkat Karthik <pallela.karthik@microchip.com>
 *
 */

#ifndef _MICROCHIP_TSN_CMDS_H_
#include <limits.h>
#ifndef BIT
#define BIT(nr) (1UL << (nr))
#endif

#ifndef GENMASK
#define GENMASK(h, l) \
(((~0UL) << (l)) & (~0UL >> (CHAR_BIT * sizeof(unsigned long) - 1 - (h))))
#endif

#define _MICROCHIP_TSN_CMDS_H_

#define ETHER_ADDR_LEN			6
#define MAX_TSN_CONFIG_SIZE		4096
#define MCHP_TSN_NUM_PRIO_QUEUES_V2	7
#define MCHP_TSN_NUM_PRIO_QUEUES_V3	8
#define MCHP_TSN_NUM_STREAM_ID_PER_Q_V3 2
#define MCHP_TSN_NUM_PRIO_QUEUES MCHP_TSN_NUM_PRIO_QUEUES_V2
#define TSN_CMD_ERR_STR_LEN		128
#define MCHP_TSN_F_QBV_V2	BIT(0)
#define MCHP_TSN_F_QBV_V3	BIT(1)
#define MCHP_TSN_NUM_PSFP_port_STREAM_ID 8
#define MCHP_TSN_NUM_PSFP_port           2

#define __packed __attribute__((packed))

struct mchp_tsn_config_cmd_resp {
	__u8  cmd;
	__u8  cmd_status;

	__be64 tsn_dev_id;
	__be16 tsn_config_size;
	__u8  cmd_status_string_avail;
	__u8  cmd_status_string[TSN_CMD_ERR_STR_LEN];
	__u8  tsn_config_data[];
} __packed;

#define MCHP_TSN_CONFIG_CMD      _IOWR('P', 1, struct mchp_tsn_config_cmd_resp)

struct mchp_tsn_config_device_info {
	__be64 tsn_dev_id;
} __packed;

struct mchp_tsn_gcl_entry {
	__be32 time_interval;
	__u8  gate_state;
} __packed;

struct mchp_tsn_config_qbv_v2 {
	__u8 initial_gate_state;
	__u8 priority_enable;
	__u8 gate_enable;
	__u8 priority_queue_enable;
	__u8 priority_queue_prios[MCHP_TSN_NUM_PRIO_QUEUES_V2];
	__u8 control_list_length;

	__be32 cycle_time;
	__be64 basetime_sec;
	__be32 basetime_nsec;
	__u8 basetime_adjust;
	struct mchp_tsn_gcl_entry gcle[];
} __packed;

struct mchp_tsn_config_qbv_streamid_v3 {
	__u8 mask;
	__u8 reserved0;
	__u8 da[ETHER_ADDR_LEN];
	__u8 reserved1;

	__be16 vid;
	__u8 pcp;
	__u8 frer;
	__u8 reserved2[2];
} __packed;

struct mchp_tsn_prioq_prio_v3 {
	struct mchp_tsn_config_qbv_streamid_v3 streamid[MCHP_TSN_NUM_STREAM_ID_PER_Q_V3];
} __packed;

struct mchp_tsn_config_qbv_v3 {
	__u8 initial_gate_state;
	__u8 priority_enable;
	__u8 gate_enable;
	__u8 priority_queue_enable;
	__u8 priority_queue_prios[MCHP_TSN_NUM_PRIO_QUEUES_V3];
	struct mchp_tsn_prioq_prio_v3 priority_queue_prios_que[MCHP_TSN_NUM_PRIO_QUEUES_V3];
	__u8 control_list_length;

	__be64 cycle_time;
	__be64 basetime_sec;
	__be32 basetime_nsec;
	__u8 basetime_adjust;

	__be32 rx_streamid_reset;
	struct mchp_tsn_gcl_entry gcle[];
} __packed;

struct mchp_tsn_caps {
	__u8 rtl_ver;
	__u8 num_queues;
	__u8 num_streamid_per_q;
	__u8 reserved0;

	__be32 features;
	__u8 reserved1[16];
} __packed;

struct mchp_tsn_config_qbu {
	__u8 pre_empt_en;

	__be16 pre_empt_size;
} __packed;

struct mchp_tsn_cbs_q_config {
	__u8 cbs_q_num;
	__u8 cbs_en;

	__be16 cbs_inc;
	__be16 cbs_dec;
	__be32 cred_min;
	__be32 cred_max;
} __packed;

struct mchp_tsn_config_qav {
	__u8 num_cbs_queues;
	struct mchp_tsn_cbs_q_config cqc[];
} __packed;

struct mchp_tsn_pfsp_stream_info {
	__u32 pfsp_packets_dropped;
	__u32 pfsp_packets_rcvd;
} __packed;

struct mchp_tsn_stream_info {
	__u32 packets_dropped;
	__u32 packets_sent;
} __packed;

struct mchp_tsn_pfsp_stream_id {
	struct mchp_tsn_pfsp_stream_info pfsp_stream_info[MCHP_TSN_NUM_PSFP_port_STREAM_ID];
} __packed;

struct mchp_tsn_priority_queue {
	struct mchp_tsn_stream_info stream_info[MCHP_TSN_NUM_STREAM_ID_PER_Q_V3];
} __packed;

struct mchp_tsn_config_statistics {
	struct mchp_tsn_priority_queue queues[MCHP_TSN_NUM_PRIO_QUEUES_V3];
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
	struct mchp_tsn_pfsp_stream_id ports[MCHP_TSN_NUM_PSFP_port];
} __packed;

struct per_port_streamid {
	__u8  mask;
	__u8  da[ETHER_ADDR_LEN];
	__be16 vid;
	__be16 max_sdu_size;
	__be32 psfp_cbs;
	__be32 psfp_ebs;
	__be32 psfp_port_cir;
	__be32 psfp_port_eir;
	__u8  psfp_port_filter_en_dis;
	__u8  psfp_port_fm_en_dis;
	__u8  psfp_port_drop_on_yellow;
	__u8  max_sdu_size_exceed;
	__u8  psfp_cfg_update;
	__u8  reserved;
} __packed;

struct psfp_per_port {
	struct per_port_streamid port_psfp[MCHP_TSN_NUM_PSFP_port_STREAM_ID];
} __packed;

struct mchp_tsn_config_qci_v3 {
	struct psfp_per_port ports[MCHP_TSN_NUM_PSFP_port];
} __packed;

struct mchp_tsn_config_qci {
	__u8 da_check;
	__u8 sa_check;
	__u8 destination_mac_addr[ETHER_ADDR_LEN];
	__u8 source_mac_addr[ETHER_ADDR_LEN];
} __packed;

struct mchp_tsn_config_misc_rx_port_id {
	__u8 port_id_rx_check;

	__be16 port_id_rx;
} __packed;

struct mchp_tsn_config_misc_ptp_tx_prioq {
	__u8 ptp_tx_prioq;
} __packed;

struct mchp_tsn_config_misc_length_deduct_byte {
	__be16 crc_deduct_len;
} __packed;

enum MCHP_TSN_CONFIG_CMDS {
	MCHP_TSN_GET_QBV = 1,
	MCHP_TSN_GET_QBU,
	MCHP_TSN_GET_QCI,
	MCHP_TSN_SET_QBV,
	MCHP_TSN_SET_QBU,
	MCHP_TSN_SET_QCI,
	MCHP_TSN_GET_MISC_RX_PORT,
	MCHP_TSN_SET_MISC_RX_PORT,
	MCHP_TSN_GET_MISC_PTP_TX_PRIOQ,
	MCHP_TSN_SET_MISC_PTP_TX_PRIOQ,
	MCHP_TSN_GET_MISC_LENGTH_DEDUCT_BYTE,
	MCHP_TSN_SET_MISC_LENGTH_DEDUCT_BYTE,
	MCHP_TSN_GET_CAPS,
	MCHP_TSN_GET_QBV_V3,
	MCHP_TSN_SET_QBV_V3,
	MCHP_TSN_SET_QAV,
	MCHP_TSN_GET_QAV,
	MCHP_TSN_GET_STATS,
	MCHP_TSN_SET_QCI_V3,
	MCHP_TSN_GET_QCI_V3,
};

#endif /* _MICROCHIP_TSN_CMDS_H_ */
