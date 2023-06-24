#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>

#include <infiniband/verbs.h>

#include <math.h>
#include <time.h>

#define ELAPSED_MS(start,stop) \
  ((((int64_t)stop.tv_sec-start.tv_sec)*1000*1000*1000+(stop.tv_nsec-start.tv_nsec))/1e6)

// ibverbs configuration parameters
#define PORT_NUM 1
#define ENTRY_SIZE 9000 /* maximum size of each send buffer */
#define SQ_NUM_DESC 512 /* maximum number of sends waiting for completion */

#define SRC_MAC  0x0c, 0x42, 0xa1, 0xa3, 0x98, 0xfe // m87, adapter index 0 (dev_list[0])
#define DST_MAC  0x04, 0x3f, 0x72, 0xa2, 0xcf, 0xea // xb-3, adapter index 2 (dev_list[2])
#define ETH_TYPE 0x08, 0x00

// IP headers are 20 bytes [RFC 791, section 3.1](https://www.rfc-editor.org/rfc/rfc791#section-3.1)
// {12 bytes of header information, 4 byte src ip, 4 byte dst ip}
// Version is 4 (IPv4), IHL=5 (4*5bytes), ToS is zero'd.
// Length field is the length of the IPv4 packet, entire packet minus ethernet header (14 bytes)
// Do not fragement (it is udp not part of fragmented stream), Protocol is UDP (0x11), 
// Time to live=0x40=d'64. Checksum was not computed and just set to {0x00, 0x00}
#define IP_HDRS  0x45, 0x00, 0x21, 0x2c, 0x00, 0x00, 0x40, 0x00, 0x40, 0x11, 0x00, 0x00
#define SRC_IP   0x0a, 0x11, 0x10, 0x0a // 10.17.16.40
#define DST_IP   0x0a, 0x11, 0x10, 0x28 // 10.17.16.10

#ifndef _C16S_T_
#define _C16S_T_
typedef struct Complex16Struct {
  int8_t re;
  int8_t im;
} complex16_t;
#endif

// FID packet definitions
#define F_ELES_PER_PKT     12    // elements per fengine
#define F_BINS_PER_PKT     25    // frequency channels per fid packet
#define F_TIME_PER_PKT     14    // time samples per packet mcount

// XB engine block definitions
#define XB_NXENGINES       50    // number of xengines
#define XB_NANTENNA        144   // elements in PAF
#define XB_NBEAMS          80    // formed output beams
#define XB_NBINS           25    // frequency channels per x
#define XB_MCNT_PER_BLOCK  336   // mcounts per block

#define FHEADER_SIZE_BYTES  64
#define FPAYLOAD_ELEMENTS   (F_ELES_PER_PKT*F_BINS_PER_PKT*F_TIME_PER_PKT)
#define FPAYLOAD_SIZE_BYTES (F_ELES_PER_PKT*F_BINS_PER_PKT*F_TIME_PER_PKT* sizeof(complex16_t))
#define FPACKET_SIZE_BYTES  (FHEADER_SIZE_BYTES + FPAYLOAD_SIZE_BYTES)

// derived parameters
// TODO if F_ELES_PER_PKT doesn't divide XB_NANTENNA
#define XB_NFENGINES         (XB_NANTENNA/F_ELES_PER_PKT)      // number of fengines sending packetized antenna data to the xengine
#define XB_TIME_PER_BLOCK    (XB_MCNT_PER_BLK*F_TIME_PER_PKT)  // time samples per mcount

// (temporary) xb header field widths, in bytes
#define FID_WIDTH  8
#define CAL_WIDTH  8
#define MCNT_WIDTH 32
#define XID_WIDTH  16
//typedef struct XB_HDRStruct {
//  uint8_t xid[XID_WIDTH];
//  uint8_t fid[FID_WIDTH];
//  uint8_t cal[CAL_WIDTH];
//  uint8_t mcnt[MCNT_WIDTH];
//} xb_hdr_t;

typedef struct __attribute__((packed)) XB_HDRStruct {
  uint16_t xid;
  uint16_t fid;
  uint16_t cal;
  uint64_t mcnt;
  uint8_t padding[50];
} xb_hdr_t;


const xb_hdr_t xb_hdr_template = {
  .xid = 0,//{ 0 },
  .fid = 0,//{ 0 },
  .cal = 26729, //{0xde, 0xad, 0xbe, 0xef, 0x21, 0x34, 0x65},
  .mcnt = 0,//{0x0}
  .padding = "hello world\n"
};

typedef struct XB_UDPStruct {
//typedef struct __attribute__((packed)) XBPayload {
  // note these could be uint16_t but we would need to use a htobe16 before
  // sending otherwise the bytes come reversed int he packet
  uint8_t src_port[2];
  uint8_t dst_port[2];
  uint8_t length[2];
  uint8_t chksum[2];
  //uint64_t xb_hdr[8];
  xb_hdr_t xb_hdr;
  complex16_t voltages[FPAYLOAD_ELEMENTS];
} xb_udp_t;

const xb_udp_t xb_template = {
  .src_port = {0xea, 0x60}, //60000,
  .dst_port = {0xea, 0x61}, //60001,
  .length = {0x21, 0x18},   //udp length = (payload length + UDP header length)
  .chksum = {0, 0},
  .xb_hdr = xb_hdr_template,
  .voltages = { 0 }
};

// use the packed attribute to ensure the exact size of the packet in bytes 
typedef struct __attribute__((packed)) NetPacket {
  uint8_t dst_mac[6];
  uint8_t src_mac[6];
  uint8_t eth_type[2];
  uint8_t ip_hdrs[12];
  uint8_t src_ip[4];
  uint8_t dst_ip[4];
  xb_udp_t payload;
} netpkt_t;

netpkt_t packet_template = { 
  .dst_mac  = {DST_MAC},
  .src_mac  = {SRC_MAC},
  .eth_type = {ETH_TYPE},
  .ip_hdrs  = {IP_HDRS},
  .src_ip   = {SRC_IP},
  .dst_ip   = {DST_IP},
  .payload  = xb_template
};

int8_t src_mac_table[XB_NFENGINES][6] = {
   {0x02, 0xa2, 0x02, 0x00, 0x02, 0x03},
   {0x02, 0xa2, 0x02, 0x00, 0x02, 0x04},
   {0x02, 0xa2, 0x02, 0x00, 0x02, 0x05},
   {0x02, 0xa2, 0x02, 0x00, 0x02, 0x06},
   {0x02, 0xa2, 0x02, 0x00, 0x02, 0x07},
   {0x02, 0xa2, 0x02, 0x00, 0x02, 0x08},
   {0x02, 0xa2, 0x02, 0x00, 0x02, 0x09},
   {0x02, 0xa2, 0x02, 0x00, 0x02, 0x0a},
   {0x02, 0xa2, 0x02, 0x00, 0x02, 0x0b},
   {0x02, 0xa2, 0x02, 0x00, 0x02, 0x0c},
   {0x02, 0xa2, 0x02, 0x00, 0x02, 0x0d},
   {0x02, 0xa2, 0x02, 0x00, 0x02, 0x0e}
};

int8_t dst_ipaddr_table[XB_NFENGINES][4] = {
  {0x0a, 0x11, 0x10, 0x0a}, // 10.17.16.10
  {0x0a, 0x11, 0x10, 0x0b}, // 10.17.16.11
  {0x0a, 0x11, 0x10, 0x0c}, // 10.17.16.12
  {0x0a, 0x11, 0x10, 0x0d}, // 10.17.16.13
  {0x0a, 0x11, 0x10, 0x0e}, // 10.17.16.14
  {0x0a, 0x11, 0x10, 0x0f}, // 10.17.16.15
  {0x0a, 0x11, 0x10, 0x10}, // 10.17.16.16
  {0x0a, 0x11, 0x10, 0x11}, // 10.17.16.17
  {0x0a, 0x11, 0x10, 0x12}, // 10.17.16.18
  {0x0a, 0x11, 0x10, 0x13}, // 10.17.16.19
  {0x0a, 0x11, 0x10, 0x14}, // 10.17.16.20
  {0x0a, 0x11, 0x10, 0x15}, // 10.17.16.21
};

int8_t dst_mac_addr[6] = {DST_MAC};

/*
 * Multiplicative LCG for generating uniform(0.0, 1.0) random numbers
 *   x_n = 7^5*x_(n-1)mod(2^31 - 1)
 *   With x seeded to 1 the 10000th x value should be 1043618065
 *   From R. Jain, "The Art of Computer Systems Performance Analysis,"
 *   John Wiley & Sons, 1991. (Page 443, Figure 26.2)
 */
double rand_val(int seed) {
  const long  a =      16807;  // Multiplier
  const long  m = 2147483647;  // Modulus
  const long  q =     127773;  // m div a
  const long  r =       2836;  // m mod a
  static long x;               // Random int value
  long        x_div_q;         // x divided by q
  long        x_mod_q;         // x modulo q
  long        x_new;           // New x value

  // Set the seed if argument is non-zero and then return zero
  if (seed > 0) {
    x = seed;
    return(0.0);
  }

  // RNG using integer arithmetic
  x_div_q = x / q;
  x_mod_q = x % q;
  x_new = (a * x_mod_q) - (r * x_div_q);
  if (x_new > 0)
    x = x_new;
  else
    x = x_new + m;

  // Return a random value between 0.0 and 1.0
  return((double) x / m);
}

/*
 * Function to generate normally distributed random variable using the Box-Muller method
 *  params:
 *    double mean, double standard deviation
 *  returns:
 *    normally distributed random variable
 */
double norm(double mean, double std_dev) {
  double   u, r, theta;           // Variables for Box-Muller method
  double   x;                     // Normal(0, 1) rv
  double   norm_rv;               // The adjusted normal rv

  //Generate u
  u = 0.0;
  while (u == 0.0)
    u = rand_val(0);

  // Compute r
  r = sqrt(-2.0 * log(u));

  // Generate theta
  theta = 0.0;
  while (theta == 0.0)
    theta = 2.0 * M_PI * rand_val(0);

  // Generate x value
  x = r * cos(theta);
  // TODO: extend for complex random numbers (compute sin value)

  // Adjust x value for specified mean and variance
  norm_rv = (x * std_dev) + mean;

  // Return the normally distributed RV value
  return(norm_rv);
}

void fillPacketDataBoxMullerRand(complex16_t *h) {
  int Nt = F_TIME_PER_PKT; // time samples per mcnt (packet)
  int Nc = F_BINS_PER_PKT; // coarse frequency channels for a packet
  int Ni = F_ELES_PER_PKT; // elements per fid

  // Fill data modeling an fengine packet destined for a particular xid.
  // Elements are the fastest moving index with time as the slowest (element x
  // freq x time)
  for (int t=0; t<Nt; t++) {      // time
    for (int c=0; c<Nc; c++) {    // frequency
      for (int i=0; i<Ni; i++) {  // elements per fid
        int eidx = i + c*Ni + t*(Nc*Ni);
        //float K = 195.0f/Nfft;
        h[eidx].re = roundf(norm(0,1));// + cosf(2*M_PI*K*t);
        h[eidx].im = roundf(norm(0,1));// + sinf(2*M_PI*K*t);
        //data[eidx].re = 1.0 + f;
        //data[eidx].im = 0.0;
      }
    }
  }
  return;
}

// signal handler
static int run = 1;
void clear_run(int sig) {
  run = 0;
  return;
}

int main() {
  // register signal handlers
  signal(SIGINT, clear_run);
  signal(SIGTERM, clear_run);

  struct ibv_device **dev_list;
  struct ibv_device *ib_dev;
  struct ibv_context *context;
  struct ibv_pd *pd;

  int ret;

  /*1. Get the list of offload capable devices */
  dev_list = ibv_get_device_list(NULL);
  if (!dev_list) {
    perror("Failed to get devices list");
    exit(1);
  }

  /* Selected IB device. You may change the code in case you have a setup with
   * more than one adapter installed. */
  ib_dev = dev_list[0];
  if (!ib_dev) {
    fprintf(stderr, "IB device not found\n");
    exit(1);
  }

  /* 2. Get the device context */
  /* Get context to device. The context is a descriptor and needed for resource
   * tracking and operations */
  context = ibv_open_device(ib_dev);
  if (!context) {
    fprintf(stderr, "Couldn't get context for %s\n", ibv_get_device_name(ib_dev));
    exit(1);
  }

  /* 3. Allocate Protection Domain */
  pd = ibv_alloc_pd(context);
  if (!pd) {
    fprintf(stderr, "Couldn't allocate PD\n");
    exit(1);
  }

  /* 4. Create Complition Queue (CQ) */
  struct ibv_cq *cq;
  cq = ibv_create_cq(context, SQ_NUM_DESC, NULL, NULL, 0);
  if (!cq) {
    fprintf(stderr, "Couldn't create CQ %d\n", errno);
    exit (1);
  }

  /* 5. Initialize QP */
  struct ibv_qp *qp;
  struct ibv_qp_init_attr qp_init_attr = {
    .qp_context = NULL,
    /* report send completion to cq */
    .send_cq = cq,
    .recv_cq = cq,
    .cap = {
      /* number of allowed outstanding sends without waiting for a completion */
      .max_send_wr = SQ_NUM_DESC,
      /* maximum number of pointers in each descriptor */
      .max_send_sge = 1,
      /* if inline maximum of payload data in the descriptors themselves */
      .max_inline_data = 512,
      .max_recv_wr = 0
    },
    .qp_type = IBV_QPT_RAW_PACKET,
  };

  /* 6. Create Queue Pair (QP) - Send Ring */
  qp = ibv_create_qp(pd, &qp_init_attr);
  if (!qp) {
    fprintf(stderr, "Couldn't create RSS QP\n");
    exit(1);
  }

  /* 7. Initialize the QP (receive ring) and assign a port */
  //struct ibv_qp_attr qp_attr;
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

  /* 8. Move the ring to ready to send in two steps (a,b) */

  /* a. Move ring state to ready to receive, this is needed to be able to move
   * ring to ready to send even if receive queue is not enabled */
  qp_flags = IBV_QP_STATE;
  qp_attr.qp_state = IBV_QPS_RTR;

  ret = ibv_modify_qp(qp, &qp_attr, qp_flags);
  if (ret < 0) {
    fprintf(stderr, "failed modify qp to receive\n");
    exit(1);
  }

  /* b. Move the ring to ready to send */
  qp_flags = IBV_QP_STATE;
  qp_attr.qp_state = IBV_QPS_RTS;

  ret = ibv_modify_qp(qp, &qp_attr, qp_flags);
  if (ret < 0) {
    fprintf(stderr, "failed modify qp to receive\n");
    exit(1);
  }

  /* 9. Allocate Memory */
  int buf_size = ENTRY_SIZE*SQ_NUM_DESC; // maximum size of data accessed directly by hw
  void *buf;

  buf = malloc(buf_size);
  if (!buf) {
    fprintf(stderr, "Coudln't allocate memory\n");
    exit(1);
  }

  int n=0;
  int curfid = 0;
  int curmcnt = 0;
  int mcnt_stop = 4*384; // send 4 blocks
  int pkt_stop = 32*384*XB_NFENGINES;
  int fid_hist[XB_NFENGINES] = {0};
  rand_val(1); // init seed

  int Nt = F_TIME_PER_PKT;
  int Nc = F_BINS_PER_PKT;
  int Ni = F_ELES_PER_PKT;
  int Nf = XB_NFENGINES;

  netpkt_t *pktbuf = (netpkt_t*) buf;
  xb_hdr_t *hdr;
  xb_udp_t *pld;
  complex16_t *data;
  for (int f=0; f<Nf; f++) { // fid
    // upfront configuration of packet for fid 'f'
    // src mac, dst mac, and fid should only need be set once
    netpkt_t *curfid_pkt = &(pktbuf[f]);
    *curfid_pkt = packet_template;
    memcpy(curfid_pkt->src_mac, src_mac_table[f], 6);
    memcpy(curfid_pkt->dst_ip, dst_ipaddr_table[f], 6);
    // simulate antenna voltage values
    pld = &(curfid_pkt->payload);
    hdr = &(pld->xb_hdr);
    //memset(hdr->fid, f, 1);
    hdr->fid = htobe16(f);
    hdr->mcnt = htobe64(curmcnt);
    data = pld->voltages;
    fillPacketDataBoxMullerRand(data);
  }

  /* 10. Register the user memory so it can be accessed by the HW directly */
  struct ibv_mr *mr;
  mr = ibv_reg_mr(pd, buf, buf_size, IBV_ACCESS_LOCAL_WRITE);

  if (!mr) {
    fprintf(stderr, "Couldn't register mr\n");
    exit(1);
  }

  //memcpy(buf, packet, sizeof(packet));

  struct ibv_sge sg_entry;
  struct ibv_send_wr wr, *bad_wr;
  int msgs_completed;
  struct ibv_wc wc;

  /* scatter/gather entry describes location and size of data to send*/
  sg_entry.addr = (uint64_t)buf;
  sg_entry.length = sizeof(netpkt_t);
  sg_entry.lkey = mr->lkey;

  memset(&wr, 0, sizeof(wr));

  /*
   * descriptor for send transaction - details:
   *   - how many pointer to data to use
   *   - if this is a single descriptor or a list (next == NULL single)
   *   - if we want inline and/or completion
   */
  wr.num_sge = 1;
  wr.sg_list = &sg_entry;
  wr.next = NULL;
  wr.opcode = IBV_WR_SEND;
  // ask for a completion every time (could init sg_all=1 instead)
  wr.send_flags = IBV_SEND_SIGNALED;

  struct timespec start, stop;
  clock_gettime(CLOCK_MONOTONIC, &start);
  /* 10. Send Operation */
  /* push first descriptor to hardware to start us before going into loop */
  wr.wr_id = n;
  ret = ibv_post_send(qp, &wr, &bad_wr);
  if (ret < 0) {
    fprintf(stderr, "failed in post send\n");
    exit(1);
  }

  n++;

  // the way this is currently implemented `n` will incremenet and the loop will
  // exit and we will never get the last work completion (wc) to update the
  // histogram  (the other thing to do would be to just update the histogram
  // after we know post_send didn't fail instead of using the wr_id from the
  // recently completed wc.
  //while(n<pkt_stop) {
  //while(curmcnt<mcnt_stop) {
  while(run) {

    msgs_completed = ibv_poll_cq(cq, 1, &wc);

    if (msgs_completed > 0) {
      //printf("completed message w/ id: %ld\n", wc.wr_id);
      int completed_fid = (wc.wr_id) % XB_NFENGINES;
      fid_hist[completed_fid] += 1;

      if (completed_fid == (XB_NFENGINES-1)) {
        // incremement curmcnt
        // (the way this works looks like it will actually send the first packet
        // of the next mcnt then quit -- probably a better way to do this)
        curmcnt++;
        // generate the next sequence of XB_NFENGINES packets
        for (int f=0; f<Nf; f++) { // fid
          netpkt_t *curfid_pkt = &(pktbuf[f]);
          // set next mcnt and simulate new antenna voltage values
          pld = &(curfid_pkt->payload);
          hdr = &(pld->xb_hdr);
          hdr->mcnt = htobe64(curmcnt);
          data = pld->voltages;
          fillPacketDataBoxMullerRand(data);
        }
      }

      // update wr to use next packet data to be sent
      curfid = n % XB_NFENGINES;

      //printf("pointing to sed packet fid: %d\n", fid);
      sg_entry.addr = (uint64_t)buf + curfid*sizeof(netpkt_t);

      /* push new wr descriptor to hardware */
      wr.wr_id = n;
      ret = ibv_post_send(qp, &wr, &bad_wr);
      if (ret < 0) {
        fprintf(stderr, "failed in post send\n");
        exit(1);
      }

      n++;

    } else if (msgs_completed < 0) {
      printf("Polling error\n");
      exit(1);
    }
  }
  clock_gettime(CLOCK_MONOTONIC, &stop);
  float total_time_ms = (float) ELAPSED_MS(start,stop);
  printf("transmit_packets() time = %f ms\n", total_time_ms);

  size_t pkts_sent = n;
  size_t total_bytes_transmitted = pkts_sent*sizeof(netpkt_t);
  float gpbs = (8.0f * total_bytes_transmitted)/total_time_ms/1e6; // (time is in ms)
  printf("estimated transmission speed: %f gpbs\n", gpbs);

  printf("{ ");
  for (int f=0; f<XB_NFENGINES; f++) {
    printf("%d ", fid_hist[f]);
  }
  printf("}\n");
  
  printf("We are done\n");

  return 0;
}
