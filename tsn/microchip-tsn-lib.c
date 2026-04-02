// SPDX-License-Identifier: MIT
/**
 * Microchip CoreTSN API Library
 *
 * Copyright (C) 2025 Microchip Technology Inc. and its subsidiaries
 *
 * Author: Pallela Venkat Karthik <pallela.karthik@microchip.com>
 *
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include "microchip-tsn-lib.h"
#include "microchip-tsn-cmds.h"

#define MAX_DEVICES 32
#define CORETSN_DRIVER_SYSFS_PATH "/sys/bus/platform/drivers/microchip-coretsn"

static int microchip_tsn_get_caps_fd(int tsn_fd, __u64 dev_id, struct mchp_tsn_caps *caps)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	__u32 alloc_size;
	int ret;

	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp) + sizeof(struct mchp_tsn_caps);
	cmd = calloc(1, alloc_size);
	if (!cmd)
		return -ENOMEM;

	cmd->cmd = MCHP_TSN_GET_CAPS;
	cmd->tsn_dev_id = htobe64(dev_id);
	cmd->tsn_config_size = htobe16(sizeof(struct mchp_tsn_caps));

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
	if (ret) {
		ret = -errno;
	} else {
		memcpy(caps, cmd->tsn_config_data, sizeof(*caps));
		ret = cmd->cmd_status;
	}

	free(cmd);
	return ret;
}

static int qbv_conf_has_v3_only_fields(const struct qbv_conf *conf)
{
	int i, j;

	if (conf->rx_streamid_reset)
		return 1;

	for (i = 0; i < MICROCHIP_TSN_NUM_PRIO_QUEUES_V3; i++) {
		for (j = 0; j < MICROCHIP_TSN_NUM_STREAM_ID_PER_Q_V3; j++) {
			if (conf->priority_queue_prios_que[i].streamid[j].mask)
				return 1;
			if (conf->priority_queue_prios_que[i].streamid[j].vid)
				return 1;
			if (conf->priority_queue_prios_que[i].streamid[j].pcp)
				return 1;
			if (conf->priority_queue_prios_que[i].streamid[j].frer)
				return 1;
		}
	}
	return 0;
}

static int get_device_list(__u64 *dev_id)
{
	DIR *dir;
	struct dirent *entry;

	const char *prefix = "mchpcoretsn";
	__u64 device_id;
	unsigned int count = 0;
	int ret;

	dir = opendir("/dev");
	if (!dir) {
		perror("opendir");
		return 1;
	}

	while ((entry = readdir(dir)) != NULL) {
		if (strncmp(entry->d_name, prefix, strlen(prefix)) == 0) {
			ret = sscanf(entry->d_name + strlen(prefix), "%llx", &device_id);
			//if (ret != 1)
			//	continue;
			*dev_id = device_id;
			//printf("%u %u\n", *dev_id, count);
			dev_id++;
			count++;
		}
	}

	closedir(dir);

	return count;
}

static int file_exists(const char *path)
{
	FILE *fp;

	fp = fopen(path, "r");
	if (fp) {
		fclose(fp);
		return 1;
	}

	return 0;
}

int microchip_tsn_get_device_list(struct microchip_tsn_device *dev, int max_devices)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_device_info *tsn_conf;
	__u32 alloc_size;
	int tsn_fd;
	int ret = 0;
	int i;
	int path_count;
	char tsn_cdev_path[32];
	__u64 dev_id[MAX_DEVICES];

	if (max_devices < 0 || max_devices > MAX_DEVICES)
		return -EINVAL;

	if (!file_exists(CORETSN_DRIVER_SYSFS_PATH))
		return -ENOENT;

	path_count = get_device_list(dev_id);

	for (i = 0; i < path_count; i++) {
		dev[i].tsn_dev_id = dev_id[i];
		snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
			 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev_id[i]);
	}

	if (ret == 0)
		ret = path_count;

	return ret;
}

int microchip_tsn_device_get_qbv_conf(struct microchip_tsn_device *dev, struct qbv_conf *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_caps caps;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i, j;
	int control_list_length;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	ret = microchip_tsn_get_caps_fd(tsn_fd, dev->tsn_dev_id, &caps);
	if (ret) {
		close(tsn_fd);
		return ret;
	}

	memset(conf, 0, sizeof(*conf));

	if (caps.rtl_ver == 2) {
		struct mchp_tsn_config_qbv_v2 *tsn_conf_v2;

		alloc_size = sizeof(struct mchp_tsn_config_cmd_resp);
		alloc_size += sizeof(struct mchp_tsn_config_qbv_v2);
		alloc_size += MAX_DEVICES * sizeof(struct mchp_tsn_gcl_entry);

		cmd = calloc(1, alloc_size);
		if (!cmd) {
			close(tsn_fd);
			return -ENOMEM;
		}

		cmd->cmd = MCHP_TSN_GET_QBV;
		cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);

		ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
		if (ret) {
			ret = -errno;
		} else {
			tsn_conf_v2 = (struct mchp_tsn_config_qbv_v2 *)cmd->tsn_config_data;
			conf->initial_gate_state = tsn_conf_v2->initial_gate_state;
			conf->priority_enable = tsn_conf_v2->priority_enable;
			conf->gate_enable = tsn_conf_v2->gate_enable;
			conf->priority_queue_enable = tsn_conf_v2->priority_queue_enable;
			memcpy(conf->priority_queue_prios,
			       tsn_conf_v2->priority_queue_prios,
			       MCHP_TSN_NUM_PRIO_QUEUES_V2);
			conf->control_list_length = tsn_conf_v2->control_list_length;
			conf->cycle_time = be32toh(tsn_conf_v2->cycle_time);
			conf->basetime_sec = be64toh(tsn_conf_v2->basetime_sec);
			conf->basetime_nsec = be32toh(tsn_conf_v2->basetime_nsec);
			conf->basetime_adjust = tsn_conf_v2->basetime_adjust;
			control_list_length = conf->control_list_length;
			for (i = 0; i < control_list_length; i++) {
				conf->gcle[i].time_interval =
			be32toh(tsn_conf_v2->gcle[i].time_interval);
				conf->gcle[i].gate_state = tsn_conf_v2->gcle[i].gate_state;
			}
			ret = cmd->cmd_status;
		}

		free(cmd);
		close(tsn_fd);
		return ret;
	}

	if (caps.rtl_ver == 3) {
		struct mchp_tsn_config_qbv_v3 *tsn_conf_v3;

		alloc_size = sizeof(struct mchp_tsn_config_cmd_resp);
		alloc_size += sizeof(struct mchp_tsn_config_qbv_v3);
		alloc_size += MAX_DEVICES * sizeof(struct mchp_tsn_gcl_entry);

		cmd = calloc(1, alloc_size);
		if (!cmd) {
			close(tsn_fd);
			return -ENOMEM;
		}

		cmd->cmd = MCHP_TSN_GET_QBV_V3;
		cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);

		ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
		if (ret) {
			ret = -errno;
		} else {
			tsn_conf_v3 = (struct mchp_tsn_config_qbv_v3 *)cmd->tsn_config_data;
			conf->initial_gate_state = tsn_conf_v3->initial_gate_state;
			conf->priority_enable = tsn_conf_v3->priority_enable;
			conf->gate_enable = tsn_conf_v3->gate_enable;
			conf->priority_queue_enable = tsn_conf_v3->priority_queue_enable;
			memcpy(conf->priority_queue_prios,
			       tsn_conf_v3->priority_queue_prios,
			       MCHP_TSN_NUM_PRIO_QUEUES_V3);

			for (i = 0; i < MCHP_TSN_NUM_PRIO_QUEUES_V3; i++) {
				for (j = 0; j < MCHP_TSN_NUM_STREAM_ID_PER_Q_V3; j++) {
					conf->priority_queue_prios_que[i].streamid[j].mask =
			tsn_conf_v3->priority_queue_prios_que[i].streamid[j].mask;
					conf->priority_queue_prios_que[i].streamid[j].vid =
			be16toh(tsn_conf_v3->priority_queue_prios_que[i].streamid[j].vid);
					conf->priority_queue_prios_que[i].streamid[j].pcp =
			tsn_conf_v3->priority_queue_prios_que[i].streamid[j].pcp;
					conf->priority_queue_prios_que[i].streamid[j].frer =
			tsn_conf_v3->priority_queue_prios_que[i].streamid[j].frer;
					memcpy(conf->priority_queue_prios_que[i].streamid[j].da,
					       tsn_conf_v3->priority_queue_prios_que[i]
							.streamid[j].da,
					       ETHER_ADDR_LEN);
				}
			}

			conf->control_list_length = tsn_conf_v3->control_list_length;
			conf->cycle_time = be64toh(tsn_conf_v3->cycle_time);
			conf->basetime_sec = be64toh(tsn_conf_v3->basetime_sec);
			conf->basetime_nsec = be32toh(tsn_conf_v3->basetime_nsec);
			conf->basetime_adjust = tsn_conf_v3->basetime_adjust;
			conf->rx_streamid_reset = be32toh(tsn_conf_v3->rx_streamid_reset);

			control_list_length = conf->control_list_length;
			for (i = 0; i < control_list_length; i++) {
				conf->gcle[i].time_interval =
			be32toh(tsn_conf_v3->gcle[i].time_interval);
				conf->gcle[i].gate_state = tsn_conf_v3->gcle[i].gate_state;
			}
			ret = cmd->cmd_status;
		}

		free(cmd);
		close(tsn_fd);
		return ret;
	}

	close(tsn_fd);
	fprintf(stderr, "Unknown CoreTSN RTL version\n");
	return -EOPNOTSUPP;
}

int microchip_tsn_device_get_qbu_conf(struct microchip_tsn_device *dev, struct qbu_conf *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_qbu *tsn_conf;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp);
	alloc_size += sizeof(struct mchp_tsn_config_qbu);

	cmd = malloc(alloc_size);
	if (!cmd)
		return -ENOMEM;

	cmd->cmd = MCHP_TSN_GET_QBU;
	cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
	if (ret) {
		ret = -errno;
	} else {
		tsn_conf = (struct mchp_tsn_config_qbu *)cmd->tsn_config_data;
		conf->pre_empt_en = tsn_conf->pre_empt_en;
		conf->pre_empt_size = be16toh(tsn_conf->pre_empt_size);
		ret = cmd->cmd_status;
	}

	free(cmd);
	close(tsn_fd);

	return ret;
}

int microchip_tsn_device_get_qci_conf(struct microchip_tsn_device *dev, struct qci_conf *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_qci *tsn_conf;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp);
	alloc_size += sizeof(struct mchp_tsn_config_qci);

	cmd = malloc(alloc_size);
	if (!cmd)
		return -ENOMEM;

	cmd->cmd = MCHP_TSN_GET_QCI;
	cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
	if (ret) {
		ret = -errno;
	} else {
		tsn_conf = (struct mchp_tsn_config_qci *)cmd->tsn_config_data;
		conf->da_check = tsn_conf->da_check;
		conf->sa_check = tsn_conf->sa_check;
		memcpy(conf->destination_mac_addr, tsn_conf->destination_mac_addr, ETHER_ADDR_LEN);
		memcpy(conf->source_mac_addr, tsn_conf->source_mac_addr, ETHER_ADDR_LEN);
		ret = cmd->cmd_status;
	}

	free(cmd);
	close(tsn_fd);

	return ret;
}

int microchip_tsn_misc_get_rx_port_id(struct microchip_tsn_device *dev,
		struct misc_rx_port_id_conf *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_misc_rx_port_id *tsn_conf;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp);
	alloc_size += sizeof(struct mchp_tsn_config_misc_rx_port_id);

	cmd = malloc(alloc_size);
	if (!cmd)
		return -ENOMEM;

	cmd->cmd = MCHP_TSN_GET_MISC_RX_PORT;
	cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
	if (ret) {
		ret = -errno;
	} else {
		tsn_conf = (struct mchp_tsn_config_misc_rx_port_id *)cmd->tsn_config_data;
		conf->port_id_rx_check = tsn_conf->port_id_rx_check;
		conf->port_id_rx = be16toh(tsn_conf->port_id_rx);
		ret = cmd->cmd_status;
	}

	free(cmd);
	close(tsn_fd);

	return ret;
}

int microchip_tsn_misc_get_length_deduct_byte(struct microchip_tsn_device *dev,
					      struct misc_length_deduct_byte_conf *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_misc_length_deduct_byte *tsn_conf;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp);
	alloc_size += sizeof(struct mchp_tsn_config_misc_length_deduct_byte);

	cmd = malloc(alloc_size);
	if (!cmd)
		return -ENOMEM;

	cmd->cmd = MCHP_TSN_GET_MISC_LENGTH_DEDUCT_BYTE;
	cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
	if (ret) {
		ret = -errno;
	} else {
		tsn_conf = (struct mchp_tsn_config_misc_length_deduct_byte *)cmd->tsn_config_data;
		conf->crc_deduct_len = be16toh(tsn_conf->crc_deduct_len);
		ret = cmd->cmd_status;
	}

	free(cmd);
	close(tsn_fd);

	return ret;
}

int microchip_tsn_misc_get_tx_ptp_prioq(struct microchip_tsn_device *dev,
					struct misc_ptp_tx_prioq_conf *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_misc_ptp_tx_prioq *tsn_conf;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp);
	alloc_size += sizeof(struct mchp_tsn_config_misc_ptp_tx_prioq);

	cmd = malloc(alloc_size);
	if (!cmd)
		return -ENOMEM;

	cmd->cmd = MCHP_TSN_GET_MISC_PTP_TX_PRIOQ;
	cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
	if (ret) {
		ret = -errno;
	} else {
		tsn_conf = (struct mchp_tsn_config_misc_ptp_tx_prioq *)cmd->tsn_config_data;
		conf->ptp_tx_prioq = tsn_conf->ptp_tx_prioq;
		ret = cmd->cmd_status;
	}

	free(cmd);
	close(tsn_fd);

	return ret;
}

int microchip_tsn_device_set_qbv_conf(struct microchip_tsn_device *dev, struct qbv_conf *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_caps caps;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i, j;
	int control_list_length;
	size_t payload_size;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	ret = microchip_tsn_get_caps_fd(tsn_fd, dev->tsn_dev_id, &caps);
	if (ret) {
		close(tsn_fd);
		return ret;
	}

	control_list_length = conf->control_list_length;

	if (caps.rtl_ver == 2) {
		struct mchp_tsn_config_qbv_v2 *tsn_conf_v2;

		if (qbv_conf_has_v3_only_fields(conf)) {
			fprintf(stderr,
				"QBV StreamID/rx_streamid_reset requires CoreTSN RTL v3 and updated software\n");
			close(tsn_fd);
			return -EOPNOTSUPP;
		}

		payload_size = sizeof(struct mchp_tsn_config_qbv_v2) +
			control_list_length * sizeof(struct mchp_tsn_gcl_entry);
		alloc_size = sizeof(struct mchp_tsn_config_cmd_resp) + payload_size;

		cmd = calloc(1, alloc_size);
		if (!cmd) {
			close(tsn_fd);
			return -ENOMEM;
		}

		cmd->cmd = MCHP_TSN_SET_QBV;
		cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);
		cmd->tsn_config_size = htobe16(payload_size);

		tsn_conf_v2 = (struct mchp_tsn_config_qbv_v2 *)cmd->tsn_config_data;
		tsn_conf_v2->initial_gate_state = conf->initial_gate_state;
		tsn_conf_v2->priority_enable = conf->priority_enable;
		tsn_conf_v2->gate_enable = conf->gate_enable;
		tsn_conf_v2->priority_queue_enable = conf->priority_queue_enable;
		memcpy(tsn_conf_v2->priority_queue_prios,
		       conf->priority_queue_prios,
		       MCHP_TSN_NUM_PRIO_QUEUES_V2);
		tsn_conf_v2->control_list_length = conf->control_list_length;
		tsn_conf_v2->cycle_time = htobe32((__u32)conf->cycle_time);
		tsn_conf_v2->basetime_sec = htobe64(conf->basetime_sec);
		tsn_conf_v2->basetime_nsec = htobe32(conf->basetime_nsec);
		tsn_conf_v2->basetime_adjust = conf->basetime_adjust;

		for (i = 0; i < control_list_length; i++) {
			tsn_conf_v2->gcle[i].time_interval = htobe32(conf->gcle[i].time_interval);
			tsn_conf_v2->gcle[i].gate_state = conf->gcle[i].gate_state;
		}

		ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
		if (ret)
			ret = -errno;
		else
			ret = cmd->cmd_status;

		if (cmd->cmd_status_string_avail)
			printf("%s\n", cmd->cmd_status_string);

		free(cmd);
		close(tsn_fd);
		return ret;
	}

	if (caps.rtl_ver == 3) {
		struct mchp_tsn_config_qbv_v3 *tsn_conf_v3;

		payload_size = sizeof(struct mchp_tsn_config_qbv_v3) +
			control_list_length * sizeof(struct mchp_tsn_gcl_entry);
		alloc_size = sizeof(struct mchp_tsn_config_cmd_resp) + payload_size;

		cmd = calloc(1, alloc_size);
		if (!cmd) {
			close(tsn_fd);
			return -ENOMEM;
		}

		cmd->cmd = MCHP_TSN_SET_QBV_V3;
		cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);
		cmd->tsn_config_size = htobe16(payload_size);

		tsn_conf_v3 = (struct mchp_tsn_config_qbv_v3 *)cmd->tsn_config_data;
		tsn_conf_v3->initial_gate_state = conf->initial_gate_state;
		tsn_conf_v3->priority_enable = conf->priority_enable;
		tsn_conf_v3->gate_enable = conf->gate_enable;
		tsn_conf_v3->priority_queue_enable = conf->priority_queue_enable;

		printf("LIB tsn_conf_v3->priority_queue_enable = 0x%02x\n",
		       tsn_conf_v3->priority_queue_enable);

		memcpy(tsn_conf_v3->priority_queue_prios,
		       conf->priority_queue_prios,
		       MCHP_TSN_NUM_PRIO_QUEUES_V3);

		for (i = 0; i < MCHP_TSN_NUM_PRIO_QUEUES_V3; i++) {
			for (j = 0; j < MCHP_TSN_NUM_STREAM_ID_PER_Q_V3; j++) {
				tsn_conf_v3->priority_queue_prios_que[i].streamid[j].mask =
					conf->priority_queue_prios_que[i].streamid[j].mask;
				memcpy(tsn_conf_v3->priority_queue_prios_que[i].streamid[j].da,
				       conf->priority_queue_prios_que[i].streamid[j].da,
				       ETHER_ADDR_LEN);
				tsn_conf_v3->priority_queue_prios_que[i].streamid[j].vid =
					htobe16(conf->priority_queue_prios_que[i].streamid[j].vid);
				tsn_conf_v3->priority_queue_prios_que[i].streamid[j].pcp =
					conf->priority_queue_prios_que[i].streamid[j].pcp;
				tsn_conf_v3->priority_queue_prios_que[i].streamid[j].frer =
					conf->priority_queue_prios_que[i].streamid[j].frer;
			}
		}

		tsn_conf_v3->control_list_length = conf->control_list_length;
		tsn_conf_v3->cycle_time = htobe64(conf->cycle_time);
		tsn_conf_v3->basetime_sec = htobe64(conf->basetime_sec);
		tsn_conf_v3->basetime_nsec = htobe32(conf->basetime_nsec);
		tsn_conf_v3->basetime_adjust = conf->basetime_adjust;
		tsn_conf_v3->rx_streamid_reset = htobe32(conf->rx_streamid_reset);

		for (i = 0; i < control_list_length; i++) {
			tsn_conf_v3->gcle[i].time_interval = htobe32(conf->gcle[i].time_interval);
			tsn_conf_v3->gcle[i].gate_state = conf->gcle[i].gate_state;
		}

		ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
		if (ret)
			ret = -errno;
		else
			ret = cmd->cmd_status;

		if (cmd->cmd_status_string_avail)
			printf("%s\n", cmd->cmd_status_string);

		free(cmd);
		close(tsn_fd);
		return ret;
	}

	close(tsn_fd);
	fprintf(stderr, "Unknown CoreTSN RTL version\n");
	return -EOPNOTSUPP;
}

int microchip_tsn_device_set_qbu_conf(struct microchip_tsn_device *dev, struct qbu_conf *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_qbu *tsn_conf;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp);
	alloc_size += sizeof(struct mchp_tsn_config_qbu);

	cmd = malloc(alloc_size);
	if (!cmd)
		return -ENOMEM;

	cmd->cmd = MCHP_TSN_SET_QBU;
	cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);
	cmd->tsn_config_size = htobe16(sizeof(struct mchp_tsn_config_qbu));

	tsn_conf = (struct mchp_tsn_config_qbu *)cmd->tsn_config_data;
	tsn_conf->pre_empt_en = conf->pre_empt_en;
	tsn_conf->pre_empt_size = htobe16(conf->pre_empt_size);

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
	if (ret)
		ret = -errno;
	else
		ret = cmd->cmd_status;

	if (cmd->cmd_status_string_avail)
		printf("%s\n", cmd->cmd_status_string);

	free(cmd);
	close(tsn_fd);

	return ret;
}

int microchip_tsn_device_set_qci_conf(struct microchip_tsn_device *dev, struct qci_conf *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_qci *tsn_conf;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp);
	alloc_size += sizeof(struct mchp_tsn_config_qci);

	cmd = malloc(alloc_size);
	if (!cmd)
		return -ENOMEM;

	cmd->cmd = MCHP_TSN_SET_QCI;
	cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);
	cmd->tsn_config_size = htobe16(sizeof(struct mchp_tsn_config_qci));

	tsn_conf = (struct mchp_tsn_config_qci *)cmd->tsn_config_data;
	tsn_conf->da_check = conf->da_check;
	tsn_conf->sa_check = conf->sa_check;
	memcpy(tsn_conf->destination_mac_addr, conf->destination_mac_addr, ETHER_ADDR_LEN);
	memcpy(tsn_conf->source_mac_addr, conf->source_mac_addr, ETHER_ADDR_LEN);

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);

	if (ret)
		ret = -errno;
	else
		ret = cmd->cmd_status;

	free(cmd);
	close(tsn_fd);

	return ret;
}

int microchip_tsn_misc_set_rx_port_id(struct microchip_tsn_device *dev,
				      struct misc_rx_port_id_conf *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_misc_rx_port_id *tsn_conf;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp);
	alloc_size += sizeof(struct mchp_tsn_config_misc_rx_port_id);

	cmd = malloc(alloc_size);
	if (!cmd)
		return -ENOMEM;

	cmd->cmd = MCHP_TSN_SET_MISC_RX_PORT;
	cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);
	cmd->tsn_config_size = htobe16(sizeof(struct mchp_tsn_config_misc_rx_port_id));

	tsn_conf = (struct mchp_tsn_config_misc_rx_port_id *)cmd->tsn_config_data;
	tsn_conf->port_id_rx_check = conf->port_id_rx_check;
	tsn_conf->port_id_rx = htobe16(conf->port_id_rx);

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
	if (ret)
		ret = -errno;
	else
		ret = cmd->cmd_status;

	free(cmd);
	close(tsn_fd);

	return ret;
}

int microchip_tsn_misc_set_tx_ptp_prioq(struct microchip_tsn_device *dev,
					struct misc_ptp_tx_prioq_conf *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_misc_ptp_tx_prioq *tsn_conf;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp);
	alloc_size += sizeof(struct mchp_tsn_config_misc_ptp_tx_prioq);

	cmd = malloc(alloc_size);
	if (!cmd)
		return -ENOMEM;

	cmd->cmd = MCHP_TSN_SET_MISC_PTP_TX_PRIOQ;
	cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);
	cmd->tsn_config_size = htobe16(sizeof(struct mchp_tsn_config_misc_ptp_tx_prioq));

	tsn_conf = (struct mchp_tsn_config_misc_ptp_tx_prioq *)cmd->tsn_config_data;
	tsn_conf->ptp_tx_prioq = conf->ptp_tx_prioq;

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
	if (ret)
		ret = -errno;
	else
		ret = cmd->cmd_status;

	free(cmd);
	close(tsn_fd);

	return ret;
}

int microchip_tsn_misc_set_length_deduct_byte(struct microchip_tsn_device *dev,
					      struct misc_length_deduct_byte_conf *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_misc_length_deduct_byte *tsn_conf;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp);
	alloc_size += sizeof(struct mchp_tsn_config_misc_length_deduct_byte);

	cmd = malloc(alloc_size);
	if (!cmd)
		return -ENOMEM;

	cmd->cmd = MCHP_TSN_SET_MISC_LENGTH_DEDUCT_BYTE;
	cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);
	cmd->tsn_config_size = htobe16(sizeof(struct mchp_tsn_config_misc_length_deduct_byte));

	tsn_conf = (struct mchp_tsn_config_misc_length_deduct_byte *)cmd->tsn_config_data;
	tsn_conf->crc_deduct_len = htobe16(conf->crc_deduct_len);

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
	if (ret)
		ret = -errno;
	else
		ret = cmd->cmd_status;

	free(cmd);
	close(tsn_fd);

	return ret;
}

int microchip_tsn_device_get_caps(struct microchip_tsn_device *dev,
				  struct microchip_tsn_caps *out)
{
	struct mchp_tsn_caps caps;
	char tsn_cdev_path[32];
	int tsn_fd;
	int ret;

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	ret = microchip_tsn_get_caps_fd(tsn_fd, dev->tsn_dev_id, &caps);
	close(tsn_fd);

	if (ret)
		return ret;

	out->rtl_ver = caps.rtl_ver;
	out->num_queues = caps.num_queues;
	out->num_streamid_per_q = caps.num_streamid_per_q;

	return 0;
}

int microchip_tsn_device_set_qav_conf(struct microchip_tsn_device *dev, struct qav_conf *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_qav *tsn_conf;
	struct mchp_tsn_caps caps;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i;
	size_t payload_size;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	ret = microchip_tsn_get_caps_fd(tsn_fd, dev->tsn_dev_id, &caps);
	if (ret) {
		close(tsn_fd);
		return ret;
	}

	if (caps.rtl_ver != 3) {
		close(tsn_fd);
		fprintf(stderr, "QAV/CBS requires CoreTSN RTL v3 and updated software\n");
		return -EOPNOTSUPP;
	}

	payload_size = sizeof(struct mchp_tsn_config_qav) +
		conf->num_cbs_queues * sizeof(struct mchp_tsn_cbs_q_config);
	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp) + payload_size;

	cmd = calloc(1, alloc_size);
	if (!cmd) {
		close(tsn_fd);
		return -ENOMEM;
	}

	cmd->cmd = MCHP_TSN_SET_QAV;
	cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);
	cmd->tsn_config_size = htobe16(payload_size);

	tsn_conf = (struct mchp_tsn_config_qav *)cmd->tsn_config_data;
	tsn_conf->num_cbs_queues = conf->num_cbs_queues;

	for (i = 0; i < conf->num_cbs_queues; i++) {
		tsn_conf->cqc[i].cbs_q_num = conf->cqc[i].cbs_q_num;
		tsn_conf->cqc[i].cbs_en = conf->cqc[i].cbs_en;
		tsn_conf->cqc[i].cbs_inc = htobe16(conf->cqc[i].cbs_inc);
		tsn_conf->cqc[i].cbs_dec = htobe16(conf->cqc[i].cbs_dec);
		tsn_conf->cqc[i].cred_min = htobe32(conf->cqc[i].cred_min);
		tsn_conf->cqc[i].cred_max = htobe32(conf->cqc[i].cred_max);
	}

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
	if (ret)
		ret = -errno;
	else
		ret = cmd->cmd_status;

	if (cmd->cmd_status_string_avail)
		printf("%s\n", cmd->cmd_status_string);

	free(cmd);
	close(tsn_fd);
	return ret;
}

int microchip_tsn_device_get_qav_conf(struct microchip_tsn_device *dev, struct qav_conf *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_qav *tsn_conf;
	struct mchp_tsn_caps caps;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	ret = microchip_tsn_get_caps_fd(tsn_fd, dev->tsn_dev_id, &caps);
	if (ret) {
		close(tsn_fd);
		return ret;
	}

	if (caps.rtl_ver != 3) {
		close(tsn_fd);
		fprintf(stderr, "QAV/CBS requires CoreTSN RTL v3 and updated software\n");
		return -EOPNOTSUPP;
	}

	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp) +
		sizeof(struct mchp_tsn_config_qav) +
		2 * sizeof(struct mchp_tsn_cbs_q_config);

	cmd = calloc(1, alloc_size);
	if (!cmd) {
		close(tsn_fd);
		return -ENOMEM;
	}

	cmd->cmd = MCHP_TSN_GET_QAV;
	cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
	if (ret) {
		ret = -errno;
	} else {
		tsn_conf = (struct mchp_tsn_config_qav *)cmd->tsn_config_data;
		conf->num_cbs_queues = tsn_conf->num_cbs_queues;
		for (i = 0; i < conf->num_cbs_queues; i++) {
			conf->cqc[i].cbs_q_num = tsn_conf->cqc[i].cbs_q_num;
			conf->cqc[i].cbs_en = tsn_conf->cqc[i].cbs_en;
			conf->cqc[i].cbs_inc = be16toh(tsn_conf->cqc[i].cbs_inc);
			conf->cqc[i].cbs_dec = be16toh(tsn_conf->cqc[i].cbs_dec);
			conf->cqc[i].cred_min = be32toh(tsn_conf->cqc[i].cred_min);
			conf->cqc[i].cred_max = be32toh(tsn_conf->cqc[i].cred_max);
		}
		ret = cmd->cmd_status;
	}

	free(cmd);
	close(tsn_fd);
	return ret;
}

int microchip_tsn_device_get_streamid_stats(struct microchip_tsn_device *dev,
					    struct tsn_statistics *stats)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_statistics *tsn_stats;
	struct mchp_tsn_caps caps;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	ret = microchip_tsn_get_caps_fd(tsn_fd, dev->tsn_dev_id, &caps);
	if (ret) {
		close(tsn_fd);
		return ret;
	}

	if (caps.rtl_ver != 3) {
		close(tsn_fd);
		fprintf(stderr, "Statistics require CoreTSN RTL v3 and updated software\n");
		return -EOPNOTSUPP;
	}

	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp) +
		sizeof(struct mchp_tsn_config_statistics);

	cmd = calloc(1, alloc_size);
	if (!cmd) {
		close(tsn_fd);
		return -ENOMEM;
	}

	cmd->cmd = MCHP_TSN_GET_STATS;
	cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
	if (ret) {
		ret = -errno;
	} else {
		tsn_stats = (struct mchp_tsn_config_statistics *)cmd->tsn_config_data;
		memcpy(stats, tsn_stats, sizeof(*stats));
		ret = cmd->cmd_status;
	}

	free(cmd);
	close(tsn_fd);
	return ret;
}

int microchip_tsn_device_set_qci_conf_v3(struct microchip_tsn_device *dev,
					 struct qci_conf_v3 *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_qci_v3 *tsn_conf;
	struct mchp_tsn_caps caps;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i, j;
	size_t payload_size;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	ret = microchip_tsn_get_caps_fd(tsn_fd, dev->tsn_dev_id, &caps);
	if (ret) {
		close(tsn_fd);
		return ret;
	}

	if (caps.rtl_ver != 3) {
		close(tsn_fd);
		fprintf(stderr, "QCI/PSFP requires CoreTSN RTL v3 and updated software\n");
		return -EOPNOTSUPP;
	}

	payload_size = sizeof(struct mchp_tsn_config_qci_v3);
	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp) + payload_size;

	cmd = calloc(1, alloc_size);
	if (!cmd) {
		close(tsn_fd);
		return -ENOMEM;
	}

	cmd->cmd = MCHP_TSN_SET_QCI_V3;
	cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);
	cmd->tsn_config_size = htobe16(payload_size);

	tsn_conf = (struct mchp_tsn_config_qci_v3 *)cmd->tsn_config_data;

	for (i = 0; i < MICROCHIP_TSN_NUM_PSFP_port; i++) {
		for (j = 0; j < MICROCHIP_TSN_NUM_PSFP_port_STREAM_ID; j++) {
			tsn_conf->ports[i].port_psfp[j].mask = conf->ports[i].port_psfp[j].mask;
			memcpy(tsn_conf->ports[i].port_psfp[j].da,
			       conf->ports[i].port_psfp[j].da,
			       ETHER_ADDR_LEN);
			tsn_conf->ports[i].port_psfp[j].vid =
				htobe16(conf->ports[i].port_psfp[j].vid);
			tsn_conf->ports[i].port_psfp[j].max_sdu_size =
				htobe16(conf->ports[i].port_psfp[j].max_sdu_size);
			tsn_conf->ports[i].port_psfp[j].psfp_cbs =
				htobe32(conf->ports[i].port_psfp[j].psfp_cbs);
			tsn_conf->ports[i].port_psfp[j].psfp_ebs =
				htobe32(conf->ports[i].port_psfp[j].psfp_ebs);
			tsn_conf->ports[i].port_psfp[j].psfp_port_cir =
				htobe32(conf->ports[i].port_psfp[j].psfp_port_cir);
			tsn_conf->ports[i].port_psfp[j].psfp_port_eir =
				htobe32(conf->ports[i].port_psfp[j].psfp_port_eir);
			tsn_conf->ports[i].port_psfp[j].psfp_port_filter_en_dis =
				conf->ports[i].port_psfp[j].psfp_port_filter_en_dis;
			tsn_conf->ports[i].port_psfp[j].psfp_port_fm_en_dis =
				conf->ports[i].port_psfp[j].psfp_port_fm_en_dis;
			tsn_conf->ports[i].port_psfp[j].psfp_port_drop_on_yellow =
				conf->ports[i].port_psfp[j].psfp_port_drop_on_yellow;
			tsn_conf->ports[i].port_psfp[j].max_sdu_size_exceed =
				conf->ports[i].port_psfp[j].max_sdu_size_exceed;
			tsn_conf->ports[i].port_psfp[j].psfp_cfg_update =
				conf->ports[i].port_psfp[j].psfp_cfg_update;
		}
	}

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
	if (ret)
		ret = -errno;
	else
		ret = cmd->cmd_status;

	if (cmd->cmd_status_string_avail)
		printf("%s\n", cmd->cmd_status_string);

	free(cmd);
	close(tsn_fd);
	return ret;
}

int microchip_tsn_device_get_qci_conf_v3(struct microchip_tsn_device *dev,
					 struct qci_conf_v3 *conf)
{
	struct mchp_tsn_config_cmd_resp *cmd;
	struct mchp_tsn_config_qci_v3 *tsn_conf;
	struct mchp_tsn_caps caps;
	__u32 alloc_size;
	int tsn_fd;
	int ret;
	int i, j;
	char tsn_cdev_path[32];

	snprintf(tsn_cdev_path, sizeof(tsn_cdev_path),
		 "%s%llx", MCHP_TSN_CDEV_PATH_PREFIX, dev->tsn_dev_id);
	tsn_fd = open(tsn_cdev_path, O_RDWR, 0666);
	if (tsn_fd < 0)
		return -errno;

	ret = microchip_tsn_get_caps_fd(tsn_fd, dev->tsn_dev_id, &caps);
	if (ret) {
		close(tsn_fd);
		return ret;
	}

	if (caps.rtl_ver != 3) {
		close(tsn_fd);
		fprintf(stderr, "QCI/PSFP requires CoreTSN RTL v3 and updated software\n");
		return -EOPNOTSUPP;
	}

	alloc_size = sizeof(struct mchp_tsn_config_cmd_resp) +
		sizeof(struct mchp_tsn_config_qci_v3);

	cmd = calloc(1, alloc_size);
	if (!cmd) {
		close(tsn_fd);
		return -ENOMEM;
	}

	cmd->cmd = MCHP_TSN_GET_QCI_V3;
	cmd->tsn_dev_id = htobe64(dev->tsn_dev_id);

	ret = ioctl(tsn_fd, MCHP_TSN_CONFIG_CMD, cmd);
	if (ret) {
		ret = -errno;
	} else {
		tsn_conf = (struct mchp_tsn_config_qci_v3 *)cmd->tsn_config_data;

		for (i = 0; i < MICROCHIP_TSN_NUM_PSFP_port; i++) {
			for (j = 0; j < MICROCHIP_TSN_NUM_PSFP_port_STREAM_ID; j++) {
				conf->ports[i].port_psfp[j].mask =
					tsn_conf->ports[i].port_psfp[j].mask;
				memcpy(conf->ports[i].port_psfp[j].da,
				       tsn_conf->ports[i].port_psfp[j].da,
				       ETHER_ADDR_LEN);
				conf->ports[i].port_psfp[j].vid =
					be16toh(tsn_conf->ports[i].port_psfp[j].vid);
				conf->ports[i].port_psfp[j].max_sdu_size =
					be16toh(tsn_conf->ports[i].port_psfp[j].max_sdu_size);
				conf->ports[i].port_psfp[j].psfp_cbs =
					be32toh(tsn_conf->ports[i].port_psfp[j].psfp_cbs);
				conf->ports[i].port_psfp[j].psfp_ebs =
					be32toh(tsn_conf->ports[i].port_psfp[j].psfp_ebs);
				conf->ports[i].port_psfp[j].psfp_port_cir =
					be32toh(tsn_conf->ports[i].port_psfp[j].psfp_port_cir);
				conf->ports[i].port_psfp[j].psfp_port_eir =
					be32toh(tsn_conf->ports[i].port_psfp[j].psfp_port_eir);
				conf->ports[i].port_psfp[j].psfp_port_filter_en_dis =
					tsn_conf->ports[i].port_psfp[j].psfp_port_filter_en_dis;
				conf->ports[i].port_psfp[j].psfp_port_fm_en_dis =
					tsn_conf->ports[i].port_psfp[j].psfp_port_fm_en_dis;
				conf->ports[i].port_psfp[j].psfp_port_drop_on_yellow =
					tsn_conf->ports[i].port_psfp[j].psfp_port_drop_on_yellow;
				conf->ports[i].port_psfp[j].max_sdu_size_exceed =
					tsn_conf->ports[i].port_psfp[j].max_sdu_size_exceed;
				conf->ports[i].port_psfp[j].psfp_cfg_update =
					tsn_conf->ports[i].port_psfp[j].psfp_cfg_update;
			}
		}

		ret = cmd->cmd_status;
	}

	free(cmd);
	close(tsn_fd);
	return ret;
}
