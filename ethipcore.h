#ifndef __ETHIPCORE_H__
#define __ETHIPCORE_H__

#define ARM_DDR_PACKETS_BUF_LN	512

struct ethipcore_priv {
	void *eth_rx_buf, 
	void *eth_tx_buf;
	phys_addr_t dma_addr_rx;
	phys_addr_t dma_addr_tx;
	unsigned int phyaddr;
	unsigned int interface;
	struct phy_device *phydev;
	struct mii_dev *bus;
	// mdio address space
	phys_addr_t mdio_base;
	// eth_ip_core address space
	phys_addr_t eth_ip_core_base;
};

// Eth IP core definitions===============================================================================================
enum{
	work_ena_inp = (1<<0), // 1- start / 0 -stop
	use_vlan = (1<<1), // If 1 => eth_ip_core uses vlan_id array for filtering packets
	ena_irq = (1<<2), // разрешить генерировнаие irq
	hsr_tagging = (1<<3), // If 1 => hsr packets tagging
	prp_tagging = (1<<4), // If 1 => prp packets tagging
	set_duplicate_accept_mode = (1<<5), // If 1 => no one packet is filtered
	set_100Mbit_ethA = (1<<6), // 6 If 1 => link is 10/100 Mbit/s; If 0 => link is 1 Gbit/s
	set_100Mbit_ethB = (1<<7), // 7 If 1 => link is 10/100 Mbit/s; If 0 => link is 1 Gbit/s
	eth_hard_reset = (1<<8), // 8 If 1 => phy reset
	ena_rcv_multicast_hsr_prp = (1<<9), // 9 If 1 => Linux will receive HSR/PRP multicast packets: 0x01,0x15,0x4E,0x00,0x01,0xXX
	ena_rcv_multicast_goose = (1<<10), // 10 If 1 => Linux will receive GOOSE multicast packets: 0x01,0x0C,0xCD,0x01,0000000Xb,0xXX
	ena_rcv_multicast_gsse = (1<<11), // 11 If 1 => Linux will receive GSSE multicast packets: 0x01,0x0C,0xCD,0x02,0000000Xb,0xXX
	ena_rcv_multicast_sv = (1<<12), // 12 If 1 => Linux will receive SV multicast packets: 0x01,0x0C,0xCD,0x04,0000000Xb,0xXX
	ena_rcv_multicast_ptp = (1<<13), // 13 If 1 => Linux will receive PTP multicast packets: 0x01,0x1B,0x19,0x00,0x00,0x00 and 0x01,0x80,0xC2,0x00,0x00,0x0E
	ena_all_multicast = (1<<14), // 14 If 1 => Linux will receive all multicast packets: <1st octet><0 bit> of dst_mac == 1
	promiscuous_mode = (1<<15) // 15 If 1 => Linux will receive all packets.
};

typedef union{
	struct{
		unsigned int cntrl; //0 ������� ������������� ���� ����������
		unsigned int ddr_rcv_pckts_wr; //1 eth_ip_core �������� ARM ��������� �� ���������� � DDR ������(RX for ARM)
		unsigned int ddr_trm_pckts_rd; //2 eth_ip_core �������� ARM ��������� ������������ �� DDR �������(TX for ARM)
		unsigned int core_id; //3
		unsigned int packets_receivedA; //4 Счетчик полученных пакетов с правильной crc по порту А
		unsigned int packets_receivedB; //5 счетчик полученных пакетов с правильной crc по порту B
		unsigned int crc_err_receivedA; //6 Счетчик отброшенных пакетов из-за неправильной crc по порту А
		unsigned int crc_err_receivedB; //7 Счетчик отброшенных пакетов из-за неправильной crc по порту B
		unsigned int err_lanid_cntA; //8 Счетчик prp пакетов с неверной lan_id (пакет из lan_b попал в сеть lan_a)
		unsigned int err_lanid_cntB; //9 Счетчик prp пакетов с неверной lan_id (пакет из lan_a попал в сеть lan_b)
		unsigned int frm_size_err_cntA; // 10 Число отброшенных пакетов по причине превышения его длины (размер принимаемого фрэйма без учета CRC и преамбулы >1520)
		unsigned int frm_size_err_cntB; // 11 Число отброшенных пакетов по причине превышения его длины (размер принимаемого фрэйма без учета CRC и преамбулы >1520)
		unsigned int queue_ovf_cntA; // 12 счетчик отброшенных пакетов по причине переполненной буфера приема A
		unsigned int queue_ovf_cntB; // 13 счетчик отброшенных пакетов по причине переполненной буфера приема B
	}rd;
	struct{
		unsigned int cntrl; //0
		unsigned int clear_irq; //1 ������� ���������� ������ � ����� ����� �����������, ����� ��� �������� ������ ����������
		unsigned int ddr_rcv_pckts_addr; //2 ��������� �� ����� ����������� �������
		unsigned int ddr_trm_pckts_addr; //3 ����������� ����� ������������ �������
		unsigned int cur_pckt_trm_pos; //4 ����� �������� ������������� ������
		unsigned int our_mac_low32; //5 младшие 32 бита собственного мак адреса
		unsigned int our_mac_hi16; //6 старшие 32 бита собственного мак адреса
		unsigned int mult_mac_low32; //7 младшие 32 бита hsr multicast мак адреса
		unsigned int mult_mac_hi16; //8 старшие 32 бита hsr multicast мак адреса
		unsigned int entry_forget_time_ms; //9 установить время "забывания" пакета для протокола hsr/prp в мс. По умолчанию в fpga установлен = 40мс
	}wr;
}*pETH_comp_struct;

#pragma pack(push,1)
typedef struct
{
	uint16_t pack_sz; // Размер пакета в байтах от MAC заголовка включительно до CRC (не включено)
	uint8_t cntrl_bits; // Контролирующие биты
	union{
		struct{
			uint16_t tod_ns_fract;
			uint32_t tod_ns;
			uint32_t tod_s_low;
			uint16_t tod_s_hi;
		};
		struct{
			uint8_t fingerprint; // !! ARM_DDR_PACKETS_BUF_LN должен быть <= 256
		};
	};
	uint8_t pad[2*INTERFACE_WIDTH_BYTES - 3 - sizeof(uint16_t)-2*sizeof(uint32_t)-sizeof(uint16_t)];
}SDescStruct;

typedef union
{
  struct{
    unsigned char c[6];
  }cc;
  struct{
    unsigned int u0;
    unsigned short u1;
  }u;
}SMac,*pSMac;

enum{
	PROT_PTPV2 = 0x88F7
};

typedef struct
{
  SMac macTo;
  SMac macFrom;
  unsigned short proto;
} SMacHdr, *pSMacHdr;

typedef struct
{
    unsigned short path_lsdu; // <15:12> path; <11:0> LSDU size
    unsigned short seq_numb;
    unsigned short src_proto;
    union{
    	struct{
    		unsigned short SupPath_SupVer;
    		unsigned short SupSeqNum;
    		unsigned char tlv1_type;
    		unsigned char tlv1_length;
    		SMac mac_danh;
    		unsigned char tlv2_type;
    		unsigned char tlv2_length;
    		SMac mac_redbox;
    		unsigned char tlv0_type;
    		unsigned char tlv0_length;
    	};
    	unsigned char cdata[HSR_PL_SIZE];
    };
}SPacketHSR, *pSPacketHSR;

typedef union
{
	struct{
		unsigned short SupPath_SupVer;
		unsigned short SupSeqNum;
		unsigned char tlv1_type;
		unsigned char tlv1_length;
		SMac mac_danh;
		unsigned char tlv2_type;
		unsigned char tlv2_length;
		SMac mac_redbox;
		unsigned char tlv0_type;
		unsigned char tlv0_length;
	};
	unsigned char cdata[HSR_PL_SIZE];
}SPacketSupervHSR, *pSPacketSupervHSR;

#define PRP_PL_SIZE	(MAX_ETH_FRAME_CNT - sizeof(SMacHdr) - 3*sizeof(short))

typedef struct
{
	SDescStruct desc;
	SMacHdr hdr;
	union{
		SPacketHSR hsr;
		SPacketSupervHSR shsr;
		SPacketArp arp;
		SPacketIcmp icmp;
		unsigned char cdata[MAX_ETH_FRAME_CNT - sizeof(SMacHdr)];
	};
}DdrEthPack, *pDdrEthPack;
#pragma pack(pop)