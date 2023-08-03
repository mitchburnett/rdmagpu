/*
 *
 *  rdmagpu - using rdma for direct memory transfer from nic to gpu
 *  Copyright (C) 2023  Mitchell C. Burnett
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.*
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>

#include <infiniband/verbs.h>
#include <cuda_runtime.h>

#define PORT_NUM 1
#define ENTRY_SIZE 9000 /* The maximum size of each received packet - set to jumbo frame */
#define RQ_NUM_DESC 512/* The maximum receive ring length without processing */

/* The MAC we are listening to. In case your setup is via a network switch,
 * you may need to change the MAC address to suit the network port MAC */
//#define DEST_MAC {0x00, 0x01, 0x02, 0x03, 0x04, 0x05}
//#define DEST_MAC {0x0c, 0x42, 0xa1, 0xa3, 0x99, 0x2e}
//#define DEST_MAC {0x0c, 0x42, 0xa1, 0xa3, 0x98, 0xfe} // m87, adapter index 0 (dev_list[0])
#define DEST_MAC  0x04, 0x3f, 0x72, 0xa2, 0xcf, 0xea // xb-3, adapter index 2 (dev_list[2])
//#define DEST_MAC {0x0c, 0x42, 0xa1, 0xa3, 0x9a, 0x06}

int main() {
  struct ibv_device **dev_list;
  struct ibv_device *ib_dev;
  struct ibv_context *context;
  struct ibv_pd *pd;
  int ret;

  /* Get the list of offload capable devices */
  dev_list = ibv_get_device_list(NULL);
  if (!dev_list) {
    perror("Failed to get IB devices list");
    exit(1);
  }

  /* 1. Get Device */
  // You may change the code in case you have a setup with more than one adapter installed.
  ib_dev = dev_list[2];
  if (!ib_dev) {
    fprintf(stderr, "IB device not found\n");
    exit(1);
  }

  /* 2. Get the device context */
  // Get context to device. The context is a descriptor and needed for
  // resource tracking and operations
  context = ibv_open_device(ib_dev);
  if (!context) {
    fprintf(stderr, "Couldn't get context for %s\n",
        ibv_get_device_name(ib_dev));
    exit(1);
  }

  /* 3. Allocate Protection Domain */
  // Allocate a protection domain to group memory regions (MR) and rings
  pd = ibv_alloc_pd(context);
  if (!pd) {
    fprintf(stderr, "Couldn't allocate PD\n");
    exit(1);
  }

  /* 4. Create Complition Queue (CQ) */
  struct ibv_cq *cq;
  cq = ibv_create_cq(context, RQ_NUM_DESC, NULL, NULL, 0);
  if (!cq) {
    fprintf(stderr, "Couldn't create CQ %d\n", errno);
    exit (1);
  }

  /* 5. Initialize QP */
  struct ibv_qp *qp;
  struct ibv_qp_init_attr qp_init_attr = {

    .qp_context = NULL,

    /* report receive completion to cq */
    .send_cq = cq,
    .recv_cq = cq,

    .cap = {
      /* no send ring */
      .max_send_wr = 0,

      /* maximum number of packets in ring */
      .max_recv_wr = RQ_NUM_DESC,

      /* only one pointer per descriptor */
      .max_recv_sge = 1,
    },

    .qp_type = IBV_QPT_RAW_PACKET,
  };

  /* 6. Create Queue Pair (QP) - Receive Ring */
  qp = ibv_create_qp(pd, &qp_init_attr);
  if (!qp) {
    fprintf(stderr, "Couldn't create RSS QP\n");
    fprintf(stderr, "%d\n", errno);
    exit(1);
  }

  /* 7. Initialize the QP (receive ring) and assign a port */
  struct ibv_qp_attr qp_attr;
  int qp_flags;

  memset(&qp_attr, 0, sizeof(qp_attr));
  qp_flags = IBV_QP_STATE | IBV_QP_PORT;
  qp_attr.qp_state = IBV_QPS_INIT;
  qp_attr.port_num = 1;
  ret = ibv_modify_qp(qp, &qp_attr, qp_flags);

  if (ret < 0) {
    fprintf(stderr, "failed modify qp to init\n");
    exit(1);
  }

  memset(&qp_attr, 0, sizeof(qp_attr));

  /* 8. Move ring state to ready to receive, this is needed in order to be able
   * to receive packets */
  qp_flags = IBV_QP_STATE;
  qp_attr.qp_state = IBV_QPS_RTR;
  ret = ibv_modify_qp(qp, &qp_attr, qp_flags);

  if (ret < 0) {
    fprintf(stderr, "failed modify qp to receive\n");
    exit(1);
  }

  /* 9. Allocate Memory */
  int buf_size = ENTRY_SIZE*RQ_NUM_DESC; /* maximum size of data to be accessed by hardware */
  void *buf = NULL;
  //buf = malloc(buf_size);
  cudaSetDevice(0);
  cudaError_t cudaStatus = cudaMallocHost((void **) &buf, buf_size);
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaMalloc failed: %s\n", cudaGetErrorString(cudaStatus));
    exit(1);
  }
  if (!buf) {
    fprintf(stderr, "Coudln't allocate memory\n");
    exit(1);
  }

  /* 10. Register the user memory so it can be accessed by the HW directly */
  struct ibv_mr *mr;
  mr = ibv_reg_mr(pd, buf, buf_size, IBV_ACCESS_LOCAL_WRITE);
  if (!mr) {
    int errsv = errno;
    fprintf(stderr, "Couldn't register MR. Error: %s\n", strerror(errsv));
    exit(1);
  }

  /* 11. Attach all buffers to the ring */
  int n;
  struct ibv_sge sg_entry;
  struct ibv_recv_wr wr, *bad_wr;

  /* pointer to packet buffer size and memory key of each packet buffer */
  sg_entry.length = ENTRY_SIZE;
  sg_entry.lkey = mr->lkey;

  /*
   *  * descriptor for receive transaction - details:
   *    - how many pointers to receive buffers to use
   *    - if this is a single descriptor or a list (next == NULL single)
   */
  wr.num_sge = 1;
  wr.sg_list = &sg_entry;
  wr.next = NULL;

  for (n = 0; n < RQ_NUM_DESC; n++) {

    /* each descriptor points to max MTU size buffer */
    sg_entry.addr = (uint64_t)buf + ENTRY_SIZE*n;

    /* index of descriptor returned when packet arrives */
    wr.wr_id = n;

    /* post receive buffer to ring */
    ibv_post_recv(qp, &wr, &bad_wr);
  }

  /* 12. Register steering rule to intercept packet to DEST_MAC and place
   * packet in ring pointed by ->qp */
  struct raw_eth_flow_attr {
    struct ibv_flow_attr attr;
    struct ibv_flow_spec_eth spec_eth;
    //struct ibv_flow_spec_ipv4 spec_ipv4;
    //struct ibv_flow_spec_tcp_udp spec_tcp_udp;
  } __attribute__((packed)) flow_attr = {

    .attr = {
      .comp_mask    = 0,
      .type         = IBV_FLOW_ATTR_NORMAL,
      .size         = sizeof(flow_attr),
      .priority     = 0,
      .num_of_specs = 1,
      //.num_of_specs = 2,
      //.num_of_specs = 3,
      .port         = PORT_NUM, // IF WE ARE ON PORT 2 of the NIC I THINK THIS WILL NEED TO BE CHANGED
      .flags        = 0,
    },

    .spec_eth = {
      .type = IBV_FLOW_SPEC_ETH,
      .size = sizeof(struct ibv_flow_spec_eth),
      .val = {
        .dst_mac    = DEST_MAC,
        //.src_mac = {0x02, 0xa2, 0x02, 0x00, 0x02, 0x03},
        .src_mac    = { 0 },
        .ether_type = 0,
        .vlan_tag   = 0,
      },
      .mask = {
        .dst_mac    = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
        .src_mac    = 0, 
        .ether_type = 0,
        .vlan_tag   = 0,
      }
    },

    //.spec_ipv4 = {
    //  .type   = IBV_FLOW_SPEC_IPV4,
    //  .size   = sizeof(struct ibv_flow_spec_ipv4),
    //  .val = {
    //    .src_ip = 0x0a111028,//   0x0B86C806,
    //    .dst_ip = 0,
    //  },
    //  .mask = {
    //    .src_ip = 0xFFFFFFFF,
    //    .dst_ip = 0,
    //  }
    //},

    //.spec_tcp_udp = {
    //  .type = IBV_FLOW_SPEC_UDP,
    //  .size = sizeof(struct ibv_flow_spec_tcp_udp),
    //  .val = {
    //    .dst_port = htobe16(60001),
    //    .src_port = 0,//htobe16(60000),
    //  },
    //  .mask = {
    //    .dst_port = 0xffff,
    //    .src_port = 0//0xffff
    //  }
    //}
  };

  /* 13. Create steering rule */
  struct ibv_flow *eth_flow;
  eth_flow = ibv_create_flow(qp, &flow_attr.attr);

  if (!eth_flow) {
    fprintf(stderr, "Couldn't attach steering flow\n");
    fprintf(stderr, "%d", errno);
    exit(1);
  }

  /* 14. Wait for CQ event upon message received, and print a message */
  int msgs_completed;
  struct ibv_wc wc;

  int pktcount = 0;
  //while(1) {
  while(pktcount < 4608) {

    /* wait for completion */
    msgs_completed = ibv_poll_cq(cq, 1, &wc);

    if (msgs_completed > 0) {
      /*
       * * completion includes:
       *   -status of descriptor
       *   -index of descriptor completing
       *   -size of the incoming packets
       */

      //printf("message %ld received size %d\n", wc.wr_id, wc.byte_len);

      sg_entry.addr = (uint64_t)buf + wc.wr_id*ENTRY_SIZE;

      pktcount++;
      wr.wr_id = wc.wr_id;

      /* after processed need to post back buffer */
      ibv_post_recv(qp, &wr, &bad_wr);

    } else if (msgs_completed < 0) {

      printf("Polling error\n");
      exit(1);
    }
  }

  //printf("We got our 4608 packets\n");

  char* hostbuf= malloc(buf_size);

  cudaStatus =  cudaMemcpy(hostbuf, buf, buf_size, cudaMemcpyDeviceToHost);
  if (cudaStatus != cudaSuccess) {
    printf("failed to copy data back from GPU\n");
    exit(1);
  }

  if(write(1, hostbuf, buf_size) == -1) {
    fprintf(stdout, "error dumping with write\n");
    return 1;
  }
  return 0;
}

