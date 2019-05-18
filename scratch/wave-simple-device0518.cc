/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mao Zhongjie <zhongjie0742@163.com>
 * 2019/05/18
 * 30nodes,6SCH,select idle channel first otherwise delete the packet
 */
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/seq-ts-header.h"
#include "ns3/wave-net-device.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/net-device.h"
#include "ns3/core-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/ptr.h"
#include "ns3/double.h"
#include "ns3/rng-seed-manager.h"

#include "ns3/internet-module.h"

#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/olsr-module.h"

#include <iostream>
#include <cmath>

#include <cstdlib>
#include <ctime>
#include <fstream>
#include <vector>
#include <string>
#define random(a,b) (rand()%(b-a+1)+a)
#define node_number 30
#define SCH_num 6
#define SCH_open_time 0.2
#define stop_time 250
#define threshold_1 1
#define threshold_2 0.5
#define threshold_3 0.3
#define threshold_4 0.3
#define QUEUE_CAP 3000
using namespace ns3;
int channel_condition[6]={0};
int channel_statistic[6][200]={0};
int collision_num=0;
std::vector<int> collision_pri(5,0);
double channel_occupancy=0;
double delay_sum=0;
double delay_sum_1=0;
double delay_sum_2=0;
double delay_sum_3=0;
double delay_sum_4=0;
int receive_num=0;
int receive_num_1=0;
int receive_num_2=0;
int receive_num_3=0;
int receive_num_4=0;
int send_num=0;
int send_num_1=0;
int send_num_2=0;
int send_num_3=0;
int send_num_4=0;
int data_priority_1=0;
int data_priority_2=0;
int data_priority_3=0;
int data_priority_4=0;
int back_off_count[node_number]={0};
int back_off_state[node_number]={0};
int node_send_flag[node_number]={0};


int outqueue_num=0;

struct packet_info {
	Ptr<Packet> pkt;
	double time;
	Ipv4Address dst;
	Ipv4Address source;
	packet_info(Ptr<Packet> &pk,double t ){
		pkt = pk;
		std::cout<<"pkt"<<std::endl;
		time = t;
	}
	packet_info(){
		pkt = Create<Packet>();
		time = 0;
	}

};

typedef struct {
    packet_info data[QUEUE_CAP];
    int front,rear;
    int used;

} SEQUEUE;

SEQUEUE SPMA_QUEUE_1[node_number];
SEQUEUE SPMA_QUEUE_2[node_number];
SEQUEUE SPMA_QUEUE_3[node_number];
SEQUEUE SPMA_QUEUE_4[node_number];

/**
 * This simulation is to show the routing service of WaveNetDevice described in IEEE 09.4.
 *
 * note: although txPowerLevel is supported now, the "TxPowerLevels"
 * attribute of YansWifiPhy is 1 which means phy devices only support 1
 * levels. Thus, if users want to control txPowerLevel, they should set
 * these attributes of YansWifiPhy by themselves..
 */



class WaveNetDeviceExample
{
public:
  void SendWsmpExample (void);

  void SendIpExample (void);

  void SendWsaExample (void);

private:
  void SendOneWsmpPacket (uint32_t channel_id, uint32_t seq, uint32_t a, uint32_t b,  packet_info& pkt);
  void SendIpPacket (uint32_t seq, bool ipv6);
  bool Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
  Ptr<Socket> SetupPacketReceive (Ipv4Address addr, Ptr<Node> node);
  bool ReceiveVsa (Ptr<const Packet>,const Address &, uint32_t, uint32_t);
  void CreateWaveNodes (void);

  NodeContainer nodes;
  NetDeviceContainer devices;
  NetDeviceContainer devices_2;
  NetDeviceContainer devices_3;
  NetDeviceContainer devices_4;
  NetDeviceContainer devices_5;
  NetDeviceContainer devices_6;
  //i add
  void GenerateTraffic(uint32_t  sender_id, uint32_t  receiver_id, uint32_t channel_id,  uint32_t priority, packet_info& pkt);
  uint32_t SCH_transform (uint32_t SCH_id);
  void channel_count (void);
  void channel_reset(uint32_t id);
  void report (void);
  SEQUEUE initQueue();
  SEQUEUE inQueue(SEQUEUE Q,packet_info x);
  SEQUEUE insertQueue(SEQUEUE Q,packet_info x);
  void outQueue(SEQUEUE& Q);
  int Queuelength(SEQUEUE Q);
//  void random_add_data_1();
//  void random_add_data_2();
//  void random_add_data_3();
//  void random_add_data_4();
  void node_send_flag_reset(uint32_t node_id);
  void send_routePacket(Ptr<Packet> packet, Ipv4Address dst, Ipv4Address source);
  void Setcallback(NodeContainer nc);
  int get_sourceid(Ipv4Address ipv4Address);
  int get_idle_channel();

};

int
WaveNetDeviceExample::get_idle_channel()
{
	for (int i=0;i<SCH_num;i++)
	{
		if (channel_condition[i]==0)
		{
			return i+1;
		}
	}
	return 0;
}

int
WaveNetDeviceExample::get_sourceid(Ipv4Address ipv4Address)
{

	uint8_t buf[4]={0};
	ipv4Address.Serialize(buf);
	int id = (int) buf[3];
//	NS_LOG_UNCOND("id"<<id);
	return id;
}

static void biiiii(Ptr<Packet> packet, Ipv4Address dst, Ipv4Address relay){
	std::cout<<"biiiii"<<std::endl;
}
void
WaveNetDeviceExample::Setcallback(NodeContainer nc){
	for(unsigned int i=0; i<nc.GetN(); i++){
		Ptr<Node> nd = nc.Get(i);
		nd->GetObject<ns3::olsr::RoutingProtocol>()->sendcallback = MakeCallback(&WaveNetDeviceExample::send_routePacket, this);
//		nd->GetObject<ns3::olsr::RoutingProtocol>()->sendcallback = MakeCallback(&biiiii);
		Ptr<Packet> pkt = Create<Packet> (100);
		Ipv4Address tmp;

		nd->GetObject<ns3::olsr::RoutingProtocol>()->sendcallback(pkt, tmp, tmp);
	}

  }
void
WaveNetDeviceExample::send_routePacket(Ptr<Packet> packet, Ipv4Address dst, Ipv4Address source){
	Ipv4Address defa;
	if(dst == defa )
		return;
 	// 1. broadcast
//	NS_LOG_UNCOND (Simulator::Now().GetSeconds()<<"OLSR node in main packet");
//	NS_LOG_UNCOND (packet->GetSize()<<",dst:"<<dst<<","<<source);
	int dstnode = get_sourceid(dst);
	if (dstnode==255)
	{
//		NS_LOG_UNCOND ("getsource");
		double time=Now().GetSeconds();
//		packet_info x(packet, time);
		packet_info x;
//		NS_LOG_UNCOND ("pkt info ini");
		x.time=time;
		x.pkt=packet;
		x.dst=dst;
		x.source=source;
//		NS_LOG_UNCOND ("copy");
		SPMA_QUEUE_1[get_sourceid(source)-1]=inQueue(SPMA_QUEUE_1[get_sourceid(source)-1],x);
//		NS_LOG_UNCOND("inque");
	}
	else {
		double time=Now().GetSeconds();
		packet_info x;
		x.time=time;
		x.pkt=packet;
		x.dst=dst;
		x.source=source;
		int rand_num= random(1,100);
		if (rand_num<20)
		{
			SPMA_QUEUE_2[get_sourceid(source)-1]=inQueue(SPMA_QUEUE_2[get_sourceid(source)-1],x);
		}
		else if (rand_num<50)
			{
				SPMA_QUEUE_3[get_sourceid(source)-1]=inQueue(SPMA_QUEUE_3[get_sourceid(source)-1],x);
			}
			else {
				SPMA_QUEUE_4[get_sourceid(source)-1]=inQueue(SPMA_QUEUE_4[get_sourceid(source)-1],x);
			}
	}
// 	std::cout<<"call back in main fun"<<std::endl;
 	// 2. data packet dst
 	//// header

 }
void
WaveNetDeviceExample::node_send_flag_reset(uint32_t node_id){
	node_send_flag[node_id]=0;
}

//void
//WaveNetDeviceExample::random_add_data_1(){
//	double seq=Now().GetSeconds();
//	int node=random(1,node_number);
//	SPMA_QUEUE_1[node-1]=inQueue(SPMA_QUEUE_1[node-1],seq);
//	data_priority_1++;
//	double interval=random(10,30)/30.0;
//	if (data_priority_1<1000)
//	{
//		Simulator::Schedule (Seconds (interval), &WaveNetDeviceExample::random_add_data_1,this);
//	}
//}
//
//void
//WaveNetDeviceExample::random_add_data_2(){
//	double seq=Now().GetSeconds();
//	int node=random(1,node_number);
//	SPMA_QUEUE_2[node-1]=inQueue(SPMA_QUEUE_2[node-1],seq);
//	data_priority_2++;
//	double interval=random(66,133)/300.0;
//	if (data_priority_2<2000)
//	{
//		Simulator::Schedule (Seconds (interval), &WaveNetDeviceExample::random_add_data_2,this);
//	}
//}
//
//void
//WaveNetDeviceExample::random_add_data_3(){
//	double seq=Now().GetSeconds();
//	int node=random(1,node_number);
//	SPMA_QUEUE_3[node-1]=inQueue(SPMA_QUEUE_3[node-1],seq);
//	data_priority_3++;
//	double interval=random(44,88)/300.0;
//	if (data_priority_3<3000)
//	{
//		Simulator::Schedule (Seconds (interval), &WaveNetDeviceExample::random_add_data_3,this);
//	}
//}
//
//void
//WaveNetDeviceExample::random_add_data_4(){
//	double seq=Now().GetSeconds();
//	int node=random(1,node_number);
//	SPMA_QUEUE_4[node-1]=inQueue(SPMA_QUEUE_4[node-1],seq);
//	data_priority_4++;
//	double interval=random(33,67)/300.0;
//	if (data_priority_4<4000)
//	{
//		Simulator::Schedule (Seconds (interval), &WaveNetDeviceExample::random_add_data_4,this);
//	}
//}

SEQUEUE
WaveNetDeviceExample::initQueue(){
    SEQUEUE Q;
    //1.初始化队列,队头指针=队尾指针=0
    Q.front=Q.rear=0;
    Q.used =0;
    return Q;
}

SEQUEUE
WaveNetDeviceExample::inQueue(SEQUEUE Q,packet_info x){
    //1.判断队列是上溢,就是队尾指针是否等于最大申请的空间
//    if(Q.rear==3000){
////    	NS_LOG_UNCOND("Up Overflow\n");
//    	Q.rear = 1;
//    }else{
         //2.从队尾插入结点
		if(Q.used == QUEUE_CAP){
			this->outQueue(Q);
		}

        Q.rear = (Q.rear+1)%QUEUE_CAP;
        Q.data[Q.rear]=x;
        Q.used ++;
//        NS_LOG_UNCOND("in  success");

   return Q;
}

SEQUEUE
WaveNetDeviceExample::insertQueue(SEQUEUE Q,packet_info x){


    //1.判断队列是上溢,就是队尾指针是否等于最大申请的空间
//    if(Q.rear==3000){
////    	NS_LOG_UNCOND("Up Overflow\n");
//    	Q.rear =1;
//    }else{
//         //2.从队插入结点
	if(Q.used == QUEUE_CAP){
		this->outQueue(Q);
	}
       Q.front = (Q.front-1)%QUEUE_CAP;
        Q.data[Q.front]=x;
        Q.used++;

   return Q;
}

void
WaveNetDeviceExample::outQueue(SEQUEUE& Q){
    //1.首先判断是否是空队列
//    if(Q.front==Q.rear){
////    	NS_LOG_UNCOND("queue is empty");
//    }else{
        //2.删除结点是从队头删除
	if(Q.used==0){
		return;
	}

        Q.front=(Q.front+1)%QUEUE_CAP;
        Q.used--;
//        NS_LOG_UNCOND("out success");

    return ;
}

int
WaveNetDeviceExample::Queuelength(SEQUEUE Q){
	return Q.used;
}

void
WaveNetDeviceExample::report(void)
{
	double average_delay=delay_sum/receive_num;
	double average_delay_1=delay_sum_1/receive_num_1;
	double average_delay_2=delay_sum_2/receive_num_2;
	double average_delay_3=delay_sum_3/receive_num_3;
	double average_delay_4=delay_sum_4/receive_num_4;

	for(int i=1; i<5; i++){
		std::cout<<"collision"<<i<<" num:"<<collision_pri[i];
	}

	std::cout <<"send_num="<<send_num<<",receive_num="<<receive_num<<",average_delay="<<average_delay<<"s"<<std::endl;
	std::cout <<"send_num_1="<<send_num_1<<",receive_num_1="<<receive_num_1<<",pdr:"<<
			double(receive_num_1)/double(send_num_1*(node_number-1))<<",average_delay_1="<<average_delay_1<<"s"<<std::endl;
	std::cout <<"send_num_2="<<send_num_2<<",receive_num_2="<<receive_num_2<<",pdr:"<<
			double(receive_num_2)/double(send_num_2)<<",average_delay_2="<<average_delay_2<<"s"<<std::endl;
	std::cout <<"send_num_3="<<send_num_3<<",receive_num_3="<<receive_num_3<<",pdr:"<<
			double(receive_num_3)/double(send_num_3)<<",average_delay_3="<<average_delay_3<<"s"<<std::endl;
	std::cout <<"send_num_4="<<send_num_4<<",receive_num_4="<<receive_num_4<<",pdr:"<<
			double(receive_num_4)/double(send_num_4)<<",average_delay_4="<<average_delay_4<<"s"<<std::endl;
}

void
WaveNetDeviceExample::channel_reset(uint32_t id)
{
	channel_condition[id-1]=0;
}

void
WaveNetDeviceExample::channel_count(void)
{
	std::cout <<"channel_count running"<<std::endl;
	//Update channel_statistic[][] and calculate the sum
	double sum=0;
	int x=0;int y=0;
	for (int i=0;i<6;i++)
	{
		x=channel_statistic[i][1];
		y=channel_statistic[i][0];
		for (int j=1;j<200;j++)
		{
			x=channel_statistic[i][j];
			channel_statistic[i][j]=y;
			y=x;
			sum=sum+channel_statistic[i][j];
		}
		channel_statistic[i][0]=channel_condition[i];
		sum=sum+channel_statistic[i][0];
	}
	double percent=0;
	if (sum==0)
	{
//		std::cout <<"sum="<<sum<<std::endl;
		channel_occupancy=percent;
	}
	else
	{
		percent=sum/1200.0;
		channel_occupancy=percent;
	}

	//check SPMA_QUEUE_1 and SPMA_QUEUE_2
	for (uint32_t i=0;i<node_number;i++)
	{
		switch (back_off_state[i]){
		case 0:{
			if (Queuelength(SPMA_QUEUE_1[i])>0)
			{
				if (channel_occupancy<threshold_1)
				{
					outQueue(SPMA_QUEUE_1[i]);
					packet_info test=SPMA_QUEUE_1[i].data[SPMA_QUEUE_1[i].front];
						uint32_t channel_id=get_idle_channel();
						uint32_t receiver_id=255;
						uint32_t priority=1;
						if (channel_id>0)
						{
							GenerateTraffic(i, receiver_id, channel_id, priority, test);
							send_num++;
							send_num_1++;
						}
						else
						{
							insertQueue(SPMA_QUEUE_1[i],test);
//							send_num_1++;
							collision_pri[priority]++;
							collision_num++;
						}
				}
				else {
					back_off_state[i]=1;
					back_off_count[i]=random(1,2);
				}
			}
			else if (Queuelength(SPMA_QUEUE_2[i])>0)
				{
					if (channel_occupancy<threshold_2)
					{
						outQueue(SPMA_QUEUE_2[i]);
						packet_info test=SPMA_QUEUE_2[i].data[SPMA_QUEUE_2[i].front];
						NS_LOG_UNCOND("test source"<<test.source<<",dst"<<test.dst);
						Ipv4Address relay = nodes.Get(i)->GetObject<ns3::olsr::RoutingProtocol>()->YRouteOutput(test.source,test.dst);
						Ipv4Address def;
						if(relay == def)
						{
//							//NS_LOG_UNCOND("route not found");
//							return;
						}
						else
						{
							NS_LOG_UNCOND("else");
							uint32_t channel_id=get_idle_channel();
							uint32_t receiver_id=get_sourceid(relay)-1;
							uint32_t priority=2;
							if (channel_id>0)
							{
								GenerateTraffic(i, receiver_id, channel_id, priority, test);
								send_num++;
								send_num_2++;
							}
							else
							{
								send_num_2++;
								collision_pri[priority]++;
								collision_num++;
							}

						}
					}
					else {
						back_off_state[i]=1;
						back_off_count[i]=random(1,2);
					}
				}
					else if (Queuelength(SPMA_QUEUE_3[i])>0)
							{
								if (channel_occupancy<threshold_3)
								{
									outQueue(SPMA_QUEUE_3[i]);
									packet_info test=SPMA_QUEUE_3[i].data[SPMA_QUEUE_3[i].front];
									Ipv4Address relay = nodes.Get(i)->GetObject<ns3::olsr::RoutingProtocol>()->YRouteOutput(test.source,test.dst);
									Ipv4Address def;
									if(relay == def)
									{
//										//NS_LOG_UNCOND("route not found");
			//							return;
									}
									else
									{
										NS_LOG_UNCOND("else");
										uint32_t channel_id=get_idle_channel();
										uint32_t receiver_id=get_sourceid(relay)-1;
										uint32_t priority=3;
										if (channel_id>0)
										{
											GenerateTraffic(i, receiver_id, channel_id, priority, test);
											send_num++;
											send_num_3++;
										}
										else
										{
											send_num_3++;
											collision_pri[priority]++;
											collision_num++;
										}
									}
								}
								else
								{
									back_off_state[i]=1;
									back_off_count[i]=random(15,25);
								}
							}
						else if (Queuelength(SPMA_QUEUE_4[i])>0)
									{
										if (channel_occupancy<threshold_4)
										{
											outQueue(SPMA_QUEUE_4[i]);
											packet_info test=SPMA_QUEUE_4[i].data[SPMA_QUEUE_4[i].front];
											Ipv4Address relay = nodes.Get(i)->GetObject<ns3::olsr::RoutingProtocol>()->YRouteOutput(test.source,test.dst);
											Ipv4Address def;
											if(relay == def)
											{
//												NS_LOG_UNCOND("route not found");
					//							return;
											}
											else
											{
												NS_LOG_UNCOND("else");
												uint32_t channel_id=get_idle_channel();
												uint32_t receiver_id=get_sourceid(relay)-1;
												uint32_t priority=4;
												if (channel_id>0)
												{
													GenerateTraffic(i, receiver_id, channel_id, priority, test);
													send_num++;
													send_num_4++;
												}
												else
												{
													send_num_4++;
													collision_pri[priority]++;
													collision_num++;
												}
											}
										}
										else {
											back_off_state[i]=1;
											back_off_count[i]=random(1,2);
										}
									}


			break;
		}
		case 1:{
			back_off_count[i]--;
			if (back_off_count[i]==0)
			{
				back_off_state[i]=2;
			}
			break;
		}
		case 2:{
			if (Queuelength(SPMA_QUEUE_1[i])>0)
			{
				if (channel_occupancy<threshold_1)
				{
					outQueue(SPMA_QUEUE_1[i]);

					packet_info test=SPMA_QUEUE_1[i].data[SPMA_QUEUE_1[i].front];



						uint32_t receiver_id=255;

						uint32_t priority=1;
						uint32_t channel_id=get_idle_channel();

						if (channel_id>0)
						{
							GenerateTraffic(i, receiver_id, channel_id, priority, test);
							send_num++;
							send_num_1++;
						}
						else
						{
//							send_num_1++;
							insertQueue(SPMA_QUEUE_1[i],test);
							collision_pri[priority]++;
							collision_num++;
						}

						back_off_state[i]=0;
				}
				else {
					back_off_state[i]=1;
					back_off_count[i]=random(1,2);
				}
			}
			else if (Queuelength(SPMA_QUEUE_2[i])>0)
				{
					if (channel_occupancy<threshold_2)
					{
						outQueue(SPMA_QUEUE_2[i]);

						packet_info test=SPMA_QUEUE_2[i].data[SPMA_QUEUE_2[i].front];

						//std::cout <<"out_seq="<<seq<<std::endl;

						Ipv4Address relay = nodes.Get(i)->GetObject<ns3::olsr::RoutingProtocol>()->YRouteOutput(test.source,test.dst);
						Ipv4Address def;
						if(relay == def)
						{
//							NS_LOG_UNCOND("//NS_LOG_UNCOND("route not found");d");
//							return;
						}else{
							NS_LOG_UNCOND("else");
							uint32_t channel_id=get_idle_channel();
							uint32_t receiver_id=get_sourceid(relay)-1;
							uint32_t priority=2;
							if (channel_id>0)
							{
								GenerateTraffic(i, receiver_id, channel_id, priority, test);
								send_num++;
								send_num_2++;
							}
							else
							{
								send_num_2++;
								collision_pri[priority]++;
								collision_num++;
							}
							back_off_state[i]=0;
						}
//						uint32_t receiver_id=random(1,node_number)-1;
//						uint32_t priority=2;
//						while (receiver_id==i)
//						{
//							receiver_id=random(1,node_number)-1;
//						}
//						GenerateTraffic(i, receiver_id, channel_id,  priority,test);
//						back_off_state[i]=0;
					}
				}
				else if (Queuelength(SPMA_QUEUE_3[i])>0)
							{
								if (channel_occupancy<threshold_3)
								{
									outQueue(SPMA_QUEUE_3[i]);

									packet_info test=SPMA_QUEUE_3[i].data[SPMA_QUEUE_3[i].front];

									//std::cout <<"out_seq="<<seq<<std::endl;

									Ipv4Address relay = nodes.Get(i)->GetObject<ns3::olsr::RoutingProtocol>()->YRouteOutput(test.source,test.dst);
									Ipv4Address def;
									if(relay == def)
									{
										//NS_LOG_UNCOND("route not found");
			//							return;
									}else{
										NS_LOG_UNCOND("else");
										uint32_t channel_id=get_idle_channel();
										uint32_t receiver_id=get_sourceid(relay)-1;
										uint32_t priority=3;
										if (channel_id>0)
										{
											GenerateTraffic(i, receiver_id, channel_id, priority, test);
											send_num++;
											send_num_3++;
										}
										else
										{
											send_num_3++;
											collision_pri[priority]++;
											collision_num++;
										}
										back_off_state[i]=0;
									}
//									uint32_t receiver_id=random(1,node_number)-1;
//									uint32_t priority=3;
//									while (receiver_id==i)
//									{
//										receiver_id=random(1,node_number)-1;
//									}
//									GenerateTraffic(i, receiver_id, channel_id, priority,test);
//									back_off_state[i]=0;
								}
							}
					else if (Queuelength(SPMA_QUEUE_4[i])>0)
								{
									if (channel_occupancy<threshold_4)
									{
										outQueue(SPMA_QUEUE_4[i]);

										packet_info test=SPMA_QUEUE_4[i].data[SPMA_QUEUE_4[i].front];

										//std::cout <<"out_seq="<<seq<<std::endl;

										Ipv4Address relay = nodes.Get(i)->GetObject<ns3::olsr::RoutingProtocol>()->YRouteOutput(test.source,test.dst);
										Ipv4Address def;
										if(relay == def)
										{
											//NS_LOG_UNCOND("route not found");
				//							return;
										}else{
											NS_LOG_UNCOND("else");
											uint32_t channel_id=get_idle_channel();
											uint32_t receiver_id=get_sourceid(relay)-1;
											uint32_t priority=4;
											if (channel_id>0)
											{
												GenerateTraffic(i, receiver_id, channel_id, priority, test);
												send_num++;
												send_num_4++;
											}
											else
											{
												send_num_4++;
												collision_pri[priority]++;
												collision_num++;
											}
											back_off_state[i]=0;
										}

									}
								}
			break;
		}
		default:break;
		}
	}

	//Simulator::Schedule (Seconds (0.55), &WaveNetDeviceExample::random_add_data_1,this);

	Simulator::Schedule (Seconds (0.1), &WaveNetDeviceExample::channel_count,this);
	std::cout <<Now ().GetSeconds ()<< "s,channel_occupancy="<<channel_occupancy<<std::endl;
}

uint32_t
WaveNetDeviceExample::SCH_transform(uint32_t SCH_id)
{
	if (SCH_id < 1 || SCH_id > 6)
	{
		std::cout << "  wrong SCH_id! SCH_id must in [1,6],use default SCH1 channel"<< std::endl;
		return 172;
	}
	else
	{
		if (SCH_id<4)
		{
			return 170+2*SCH_id;
		}
		else
		{
			return 170+2*SCH_id+2;
		}
	}
}

void
WaveNetDeviceExample::GenerateTraffic(uint32_t  sender_id, uint32_t  receiver_id, uint32_t channel_id, uint32_t priority, packet_info& pkt)
{
	if (channel_condition[channel_id-1]==0)
	{
		channel_condition[channel_id-1]=1;
		node_send_flag[sender_id]=1;
		switch (channel_id)
		{
		case 1:
			{std::cout << "case 1"<<std::endl;
			if (priority==1)
			{
				uint32_t channel_number = WaveNetDeviceExample::SCH_transform (channel_id);
				const SchInfo schInfo = SchInfo (channel_number, true, EXTENDED_ALTERNATING);
				for (int i=0;i<node_number;i++)
				{
					Ptr<WaveNetDevice>  device = DynamicCast<WaveNetDevice> (devices.Get (i));
					Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch,device,schInfo);
					Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, device, channel_number);

				}
//				NS_LOG_UNCOND("channe id"<< channel_id<<","<<pkt.pkt->GetSize());
				Simulator::Schedule (Seconds (0.01), &WaveNetDeviceExample::SendOneWsmpPacket,  this, channel_id,  sender_id, receiver_id, priority, pkt);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::channel_reset, this, channel_id);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::node_send_flag_reset, this, sender_id);
//				std::cout << Now ().GetSeconds () << "s,"<< " sender_id: "<< sender_id << " is trying to transmit to recv_id:"<<receiver_id<<" at channel_id:"<< channel_id<<std::endl;
			}
			else {
				Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (sender_id));
				Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (receiver_id));

				uint32_t channel_number = WaveNetDeviceExample::SCH_transform (channel_id);

				const SchInfo schInfo = SchInfo (channel_number, true, EXTENDED_ALTERNATING);
				Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch,sender,schInfo);

				Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch, receiver, schInfo);
				Simulator::Schedule (Seconds (0.01), &WaveNetDeviceExample::SendOneWsmpPacket,  this, channel_id,  sender_id, receiver_id, priority,pkt);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, sender, channel_number);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, receiver, channel_number);
				//reset the channel  condition when the channel is stop
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::channel_reset, this, channel_id);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::node_send_flag_reset, this, sender_id);
				std::cout << Now ().GetSeconds () << "s,"<< " sender_id: "<< sender_id << " is trying to transmit to recv_id:"<<receiver_id<<" at channel_id:"<< channel_id<<std::endl;
			}

			break;
			}
		case 2:
			{std::cout << "case 2"<<std::endl;
			if (priority==1)
				{
					uint32_t channel_number = WaveNetDeviceExample::SCH_transform (channel_id);
					const SchInfo schInfo = SchInfo (channel_number, true, EXTENDED_ALTERNATING);
					for (int i=0;i<node_number;i++)
					{
						Ptr<WaveNetDevice>  device = DynamicCast<WaveNetDevice> (devices_2.Get (i));
						Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch,device,schInfo);
						Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, device, channel_number);
					}
					Simulator::Schedule (Seconds (0.01), &WaveNetDeviceExample::SendOneWsmpPacket,  this, channel_id,  sender_id, receiver_id, priority, pkt);
					Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::channel_reset, this, channel_id);
					Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::node_send_flag_reset, this, sender_id);
					std::cout << Now ().GetSeconds () << "s,"<< " sender_id: "<< sender_id << " is trying to transmit to recv_id:"<<receiver_id<<" at channel_id:"<< channel_id<<std::endl;
				}
				else {
					Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices_2.Get (sender_id));
					Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices_2.Get (receiver_id));

					uint32_t channel_number = WaveNetDeviceExample::SCH_transform (channel_id);

					const SchInfo schInfo = SchInfo (channel_number, true, EXTENDED_ALTERNATING);
					Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch,sender,schInfo);

					Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch, receiver, schInfo);
					Simulator::Schedule (Seconds (0.01), &WaveNetDeviceExample::SendOneWsmpPacket,  this, channel_id,  sender_id, receiver_id, priority,pkt);
					Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, sender, channel_number);
					Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, receiver, channel_number);
					//reset the channel  condition when the channel is stop
					Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::channel_reset, this, channel_id);
					Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::node_send_flag_reset, this, sender_id);
					std::cout << Now ().GetSeconds () << "s,"<< " sender_id: "<< sender_id << " is trying to transmit to recv_id:"<<receiver_id<<" at channel_id:"<< channel_id<<std::endl;
				}
			break;
			}
		case 3:
			{std::cout << "case 3"<<std::endl;
			if (priority==1)
			{
				uint32_t channel_number = WaveNetDeviceExample::SCH_transform (channel_id);
				const SchInfo schInfo = SchInfo (channel_number, true, EXTENDED_ALTERNATING);
				for (int i=0;i<node_number;i++)
				{
					Ptr<WaveNetDevice>  device = DynamicCast<WaveNetDevice> (devices_3.Get (i));
					Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch,device,schInfo);
					Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, device, channel_number);
				}
				Simulator::Schedule (Seconds (0.01), &WaveNetDeviceExample::SendOneWsmpPacket,  this, channel_id, sender_id, receiver_id, priority, pkt);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::channel_reset, this, channel_id);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::node_send_flag_reset, this, sender_id);
				std::cout << Now ().GetSeconds () << "s,"<< " sender_id: "<< sender_id << " is trying to transmit to recv_id:"<<receiver_id<<" at channel_id:"<< channel_id<<std::endl;
			}
			else {
				Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices_3.Get (sender_id));
				Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices_3.Get (receiver_id));

				uint32_t channel_number = WaveNetDeviceExample::SCH_transform (channel_id);

				const SchInfo schInfo = SchInfo (channel_number, true, EXTENDED_ALTERNATING);
				Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch,sender,schInfo);

				Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch, receiver, schInfo);
				Simulator::Schedule (Seconds (0.01), &WaveNetDeviceExample::SendOneWsmpPacket,  this, channel_id,  sender_id, receiver_id, priority,pkt);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, sender, channel_number);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, receiver, channel_number);
				//reset the channel  condition when the channel is stop
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::channel_reset, this, channel_id);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::node_send_flag_reset, this, sender_id);
				std::cout << Now ().GetSeconds () << "s,"<< " sender_id: "<< sender_id << " is trying to transmit to recv_id:"<<receiver_id<<" at channel_id:"<< channel_id<<std::endl;
			}
			break;
			}
		case 4:
			{std::cout << "case 4"<<std::endl;
			if (priority==1)
			{
				uint32_t channel_number = WaveNetDeviceExample::SCH_transform (channel_id);
				const SchInfo schInfo = SchInfo (channel_number, true, EXTENDED_ALTERNATING);
				for (int i=0;i<node_number;i++)
				{
					Ptr<WaveNetDevice>  device = DynamicCast<WaveNetDevice> (devices_4.Get (i));
					Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch,device,schInfo);
					Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, device, channel_number);
				}
				Simulator::Schedule (Seconds (0.01), &WaveNetDeviceExample::SendOneWsmpPacket,  this, channel_id,  sender_id, receiver_id, priority, pkt);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::channel_reset, this, channel_id);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::node_send_flag_reset, this, sender_id);
				std::cout << Now ().GetSeconds () << "s,"<< " sender_id: "<< sender_id << " is trying to transmit to recv_id:"<<receiver_id<<" at channel_id:"<< channel_id<<std::endl;
			}
			else {
				Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices_4.Get (sender_id));
				Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices_4.Get (receiver_id));

				uint32_t channel_number = WaveNetDeviceExample::SCH_transform (channel_id);

				const SchInfo schInfo = SchInfo (channel_number, true, EXTENDED_ALTERNATING);
				Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch,sender,schInfo);

				Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch, receiver, schInfo);
				Simulator::Schedule (Seconds (0.01), &WaveNetDeviceExample::SendOneWsmpPacket,  this, channel_id, sender_id, receiver_id, priority,pkt);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, sender, channel_number);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, receiver, channel_number);
				//reset the channel  condition when the channel is stop
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::channel_reset, this, channel_id);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::node_send_flag_reset, this, sender_id);
				std::cout << Now ().GetSeconds () << "s,"<< " sender_id: "<< sender_id << " is trying to transmit to recv_id:"<<receiver_id<<" at channel_id:"<< channel_id<<std::endl;
			}
			break;
			}
		case 5:
			{std::cout << "case 5"<<std::endl;
			if (priority==1)
			{
				uint32_t channel_number = WaveNetDeviceExample::SCH_transform (channel_id);
				const SchInfo schInfo = SchInfo (channel_number, true, EXTENDED_ALTERNATING);
				for (int i=0;i<node_number;i++)
				{
					Ptr<WaveNetDevice>  device = DynamicCast<WaveNetDevice> (devices_5.Get (i));
					Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch,device,schInfo);
					Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, device, channel_number);
				}
				Simulator::Schedule (Seconds (0.01), &WaveNetDeviceExample::SendOneWsmpPacket,  this, channel_id,  sender_id, receiver_id, priority, pkt);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::channel_reset, this, channel_id);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::node_send_flag_reset, this, sender_id);
				std::cout << Now ().GetSeconds () << "s,"<< " sender_id: "<< sender_id << " is trying to transmit to recv_id:"<<receiver_id<<" at channel_id:"<< channel_id<<std::endl;
			}
			else {
				Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices_5.Get (sender_id));
				Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices_5.Get (receiver_id));

				uint32_t channel_number = WaveNetDeviceExample::SCH_transform (channel_id);

				const SchInfo schInfo = SchInfo (channel_number, true, EXTENDED_ALTERNATING);
				Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch,sender,schInfo);

				Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch, receiver, schInfo);
				Simulator::Schedule (Seconds (0.01), &WaveNetDeviceExample::SendOneWsmpPacket,  this, channel_id, sender_id, receiver_id, priority,pkt);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, sender, channel_number);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, receiver, channel_number);
				//reset the channel  condition when the channel is stop
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::channel_reset, this, channel_id);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::node_send_flag_reset, this, sender_id);
				std::cout << Now ().GetSeconds () << "s,"<< " sender_id: "<< sender_id << " is trying to transmit to recv_id:"<<receiver_id<<" at channel_id:"<< channel_id<<std::endl;
			}
			break;
			}
		case 6:
			{std::cout << "case 6"<<std::endl;
			if (priority==1)
			{
				uint32_t channel_number = WaveNetDeviceExample::SCH_transform (channel_id);
				const SchInfo schInfo = SchInfo (channel_number, true, EXTENDED_ALTERNATING);
				for (int i=0;i<node_number;i++)
				{
					Ptr<WaveNetDevice>  device = DynamicCast<WaveNetDevice> (devices_6.Get (i));
					Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch,device,schInfo);
					Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, device, channel_number);
				}
				Simulator::Schedule (Seconds (0.01), &WaveNetDeviceExample::SendOneWsmpPacket,  this, channel_id,  sender_id, receiver_id, priority, pkt);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::channel_reset, this, channel_id);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::node_send_flag_reset, this, sender_id);
				std::cout << Now ().GetSeconds () << "s,"<< " sender_id: "<< sender_id << " is trying to transmit to recv_id:"<<receiver_id<<" at channel_id:"<< channel_id<<std::endl;
			}
			else {
				Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices_6.Get (sender_id));
				Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices_6.Get (receiver_id));

				uint32_t channel_number = WaveNetDeviceExample::SCH_transform (channel_id);

				const SchInfo schInfo = SchInfo (channel_number, true, EXTENDED_ALTERNATING);
				Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch,sender,schInfo);

				Simulator::Schedule (Seconds (0), &WaveNetDevice::StartSch, receiver, schInfo);
				Simulator::Schedule (Seconds (0.01), &WaveNetDeviceExample::SendOneWsmpPacket,  this, channel_id,  sender_id, receiver_id, priority,pkt);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, sender, channel_number);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDevice::StopSch, receiver, channel_number);
				//reset the channel  condition when the channel is stop
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::channel_reset, this, channel_id);
				Simulator::Schedule (Seconds (SCH_open_time), &WaveNetDeviceExample::node_send_flag_reset, this, sender_id);
				std::cout << Now ().GetSeconds () << "s,"<< " sender_id: "<< sender_id << " is trying to transmit to recv_id:"<<receiver_id<<" at channel_id:"<< channel_id<<std::endl;
			}
			break;
			}
		default:{std::cout << "wrong channel id!"<<std::endl;break;}
		}

	}
	else {
		collision_num++;
		std::cout << Now ().GetSeconds () << "s,"<< " sender_id: "<< sender_id << " have a collision trying to transmit to recv_id:"<<receiver_id<<" at channel_id:"<< channel_id<<std::endl;

	}
//	uint32_t traffic_num=count-1;
//	std::cout << "traffic_num="<<traffic_num<<std::endl;
//	if (traffic_num>0)
//	{
//		uint32_t new_send_id=random(0,4);
//		uint32_t new_recv_id=random(0,4);
//		while (new_send_id==new_recv_id)
//		{
//			new_send_id = random(0,4);
//		}
//		unsigned int new_channel_id=random(1,6);
////		double min = 1;
////		double max = 2;
////		Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
////		x->SetAttribute ("Min", DoubleValue (min));
////		x->SetAttribute ("Max", DoubleValue (max));
//		double interval = random(3,13);
//		interval=interval/100;
//		Simulator::Schedule (Seconds (interval), &WaveNetDeviceExample::GenerateTraffic, this, new_send_id, new_recv_id, new_channel_id,traffic_num);
//
//	}
}

void
WaveNetDeviceExample::CreateWaveNodes (void)
{
  nodes = NodeContainer ();
  nodes.Create (node_number);

//  MobilityHelper mobility;
//  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
//  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (1.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (2.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (3.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (4.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (5.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (6.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (7.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (8.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (9.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (10.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (11.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (12.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (13.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (14.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (15.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (16.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (17.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (18.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (19.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (20.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (21.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (22.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (23.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (24.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (25.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (26.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (27.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (28.0, 0.0, 0.0));
//  positionAlloc->Add (Vector (29.0, 0.0, 0.0));
//  mobility.SetPositionAllocator (positionAlloc);
//  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
//  mobility.Install (nodes);


  MobilityHelper mobilityAdhoc;
    int64_t streamIndex = 0; // used to get consistent mobility across scenarios

    ObjectFactory pos;
    pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
    pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"));
    pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"));

    Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
    streamIndex += taPositionAlloc->AssignStreams (streamIndex);

    mobilityAdhoc.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

//    std::stringstream ssSpeed;
//    ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";
//    std::stringstream ssPause;
//    ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";
//    mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
//                                    "Speed", StringValue (ssSpeed.str ()),
//                                    "Pause", StringValue (ssPause.str ()),
//                                    "PositionAllocator", PointerValue (taPositionAlloc));
    mobilityAdhoc.SetPositionAllocator (taPositionAlloc);
    mobilityAdhoc.Install (nodes);
//    for(int i=0;i<nWifis;i++)
//    {
// 	   mobilityAdhoc.Install (adhocNodes.Get(i));
// 	   streamIndex += mobilityAdhoc.AssignStreams (adhocNodes.Get(i), streamIndex);
//    }

  YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default ();
  waveChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",
  				DoubleValue(200));
  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  wavePhy.SetChannel (waveChannel.Create ());
  wavePhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
  devices = waveHelper.Install (wavePhy, waveMac, nodes);
  devices_2 = waveHelper.Install (wavePhy, waveMac, nodes);
  devices_3 = waveHelper.Install (wavePhy, waveMac, nodes);
  devices_4 = waveHelper.Install (wavePhy, waveMac, nodes);
  devices_5 = waveHelper.Install (wavePhy, waveMac, nodes);
  devices_6 = waveHelper.Install (wavePhy, waveMac, nodes);

  	OlsrHelper olsr;
	Ipv4StaticRoutingHelper staticRouting;

	Ipv4ListRoutingHelper list;
	list.Add (staticRouting, 0);
	list.Add (olsr, 10);

	InternetStackHelper stack;
	stack.SetRoutingHelper (list);
	stack.Install (nodes);
	Ipv4AddressHelper address;
	address.SetBase ("10.1.1.0", "255.255.255.0");
	Ipv4InterfaceContainer interfaces;
	interfaces = address.Assign (devices);
	std::cout<<"adsasdas "<<std::endl;
	Setcallback(nodes);
	std::cout<<"adsasda1 "<<std::endl;
  for (uint32_t i = 0; i != devices.GetN (); ++i)
    {
      Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (devices.Get (i));
      device->SetReceiveCallback (MakeCallback (&WaveNetDeviceExample::Receive, this));
      device->SetWaveVsaCallback (MakeCallback  (&WaveNetDeviceExample::ReceiveVsa, this));
    }

  for (uint32_t i = 0; i != devices_2.GetN (); ++i)
    {
      Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (devices_2.Get (i));
      device->SetReceiveCallback (MakeCallback (&WaveNetDeviceExample::Receive, this));
      device->SetWaveVsaCallback (MakeCallback  (&WaveNetDeviceExample::ReceiveVsa, this));
    }
  for (uint32_t i = 0; i != devices_3.GetN (); ++i)
    {
      Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (devices_3.Get (i));
      device->SetReceiveCallback (MakeCallback (&WaveNetDeviceExample::Receive, this));
      device->SetWaveVsaCallback (MakeCallback  (&WaveNetDeviceExample::ReceiveVsa, this));
    }
  for (uint32_t i = 0; i != devices_4.GetN (); ++i)
    {
      Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (devices_4.Get (i));
      device->SetReceiveCallback (MakeCallback (&WaveNetDeviceExample::Receive, this));
      device->SetWaveVsaCallback (MakeCallback  (&WaveNetDeviceExample::ReceiveVsa, this));
    }
  for (uint32_t i = 0; i != devices_5.GetN (); ++i)
    {
      Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (devices_5.Get (i));
      device->SetReceiveCallback (MakeCallback (&WaveNetDeviceExample::Receive, this));
      device->SetWaveVsaCallback (MakeCallback  (&WaveNetDeviceExample::ReceiveVsa, this));
    }
  for (uint32_t i = 0; i != devices_6.GetN (); ++i)
    {
      Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (devices_6.Get (i));
      device->SetReceiveCallback (MakeCallback (&WaveNetDeviceExample::Receive, this));
      device->SetWaveVsaCallback (MakeCallback  (&WaveNetDeviceExample::ReceiveVsa, this));
    }

  Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("64"));
  std::string rate ("20kbps");
  Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (rate));

  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
    onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
    onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=10.0]"));

    int nSinks =10;
    int tmp =0;
    for (int i = 0; i < nSinks; i++)
      {
        Ptr<Socket> sink = SetupPacketReceive (interfaces.GetAddress (i), nodes.Get (i));
        int port =9;
        AddressValue remoteAddress (InetSocketAddress (interfaces.GetAddress (i), port));
        onoff1.SetAttribute ("Remote", remoteAddress);

        Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
        ApplicationContainer temp = onoff1.Install (nodes.Get (i + nSinks));
        temp.Start (Seconds (var->GetValue (30.0,40.0)));
        temp.Stop (Seconds (stop_time));
      }
  // Tracing
//  wavePhy.EnablePcap ("wave-simple-device", devices);
//  wavePhy.EnablePcap ("wave-simple-device", devices_2);
//  wavePhy.EnablePcap ("wave-simple-device", devices_3);
//  wavePhy.EnablePcap ("wave-simple-device", devices_4);
//  wavePhy.EnablePcap ("wave-simple-device", devices_5);
//  wavePhy.EnablePcap ("wave-simple-device", devices_6);

  //
  for (int i=0;i<node_number;i++)
  {

	  SPMA_QUEUE_1[i]=initQueue();
	  SPMA_QUEUE_2[i]=initQueue();
	  SPMA_QUEUE_3[i]=initQueue();
	  SPMA_QUEUE_4[i]=initQueue();
  }
  std::cout <<"createnode end"<<std::endl;
//  Simulator::Schedule (Seconds (1), &WaveNetDeviceExample::random_add_data_1, this);
//  Simulator::Schedule (Seconds (1), &WaveNetDeviceExample::random_add_data_2, this);
//  Simulator::Schedule (Seconds (1), &WaveNetDeviceExample::random_add_data_3, this);
//  Simulator::Schedule (Seconds (1), &WaveNetDeviceExample::random_add_data_4, this);
}
Ptr<Socket>
WaveNetDeviceExample::SetupPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  int port =9;
  InetSocketAddress local = InetSocketAddress (addr, port);
  sink->Bind (local);
//  sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceivePacket, this));

  return sink;
}

bool
WaveNetDeviceExample::Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  SeqTsHeader seqTs;
  Ptr<Packet> p1=pkt->Copy();
  p1->PeekHeader (seqTs);
  p1->RemoveHeader(seqTs);
  double born_time=seqTs.GetSeq ()/100.0;

  delay_sum=delay_sum+Now ().GetSeconds ()-born_time;
  receive_num=receive_num+1;
  //if olsr packet olsr ->recv
  //
  switch (seqTs.GetPriority ()){
	   case 1:
		  {
			  delay_sum_1=delay_sum_1+Now ().GetSeconds ()-born_time;
			  receive_num_1=receive_num_1+1;
			  uint8_t buf[4]={10,1,1,100};
			  Ipv4Address source;
			  buf[3]=seqTs.GetSource();
			  source=Ipv4Address::Deserialize(buf);


			  dev->GetNode()->GetObject<ns3::olsr::RoutingProtocol>()->YRecvOlsr(p1, source);
			  break;
		  }
	   case 2:
		  {
			  delay_sum_2=delay_sum_2+Now ().GetSeconds ()-born_time;
			  receive_num_2=receive_num_2+1;
			  break;
		  }
	   case 3:
		  {
			  delay_sum_3=delay_sum_3+Now ().GetSeconds ()-born_time;
			  receive_num_3=receive_num_3+1;
			  break;
		  }
	   case 4:
		  {
			  delay_sum_4=delay_sum_4+Now ().GetSeconds ()-born_time;
			  receive_num_4=receive_num_4+1;
			  break;
		  }
	   default:break;
  }



  std::cout << "  source = "<< seqTs.GetSource() << std::endl;
//            << "  sequence = " << seqTs.GetSeq () << "," << std::endl
//			<< "  priority = " << seqTs.GetPriority () << "," << std::endl
//            << "  sendTime = " << seqTs.GetTs ().GetSeconds () << "s," << std::endl
//            << "  recvTime = " << Now ().GetSeconds () << "s," << std::endl
//            << "  protocol = 0x" << std::hex << mode << std::dec  << std::endl
//			<< "  delay_sum = " << delay_sum << std::endl;
  return true;
}

void
WaveNetDeviceExample::SendOneWsmpPacket  (uint32_t channel_id, uint32_t a, uint32_t b, uint32_t priority, packet_info& pkt)
{
  switch (channel_id)
  {
	  case 1:
	  {
//		  NS_LOG_UNCOND("c1 send one wsm pkt");
		  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (a));
//		  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (b));
		  const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
		  Mac48Address bssWildcard = Mac48Address::GetBroadcast ();
		  uint32_t channel = WaveNetDeviceExample::SCH_transform (channel_id);
		  const TxInfo txInfo = TxInfo (channel);
//		  Ptr<Packet> p  = Create<Packet> (100);
		  SeqTsHeader seqTs;
		  uint32_t seq=pkt.time*100;
		  seqTs.SetSeq (seq);
		  seqTs.SetPriority (priority);
		  seqTs.SetSource (a+1);
		  pkt.pkt->AddHeader (seqTs);
		  sender->SendX  (pkt.pkt, bssWildcard, WSMP_PROT_NUMBER, txInfo);
//		  NS_LOG_UNCOND("c1 send one wsm pkt fin");
		  break;
	  }
	  case 2:
	  {
//		  NS_LOG_UNCOND("c2 send one wsm pkt");
		  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices_2.Get (a));
//		  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices_2.Get (b));
		  const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
		  Mac48Address bssWildcard = Mac48Address::GetBroadcast ();
		  uint32_t channel = WaveNetDeviceExample::SCH_transform (channel_id);
		  const TxInfo txInfo = TxInfo (channel);
//		  Ptr<Packet> p  = Create<Packet> (100);
		  SeqTsHeader seqTs;
		  uint32_t seq=pkt.time*100;
		  seqTs.SetSeq (seq);
		  seqTs.SetPriority (priority);
		  seqTs.SetSource (a+1);
		  pkt.pkt->AddHeader (seqTs);
		  sender->SendX  (pkt.pkt, bssWildcard, WSMP_PROT_NUMBER, txInfo);
//		  NS_LOG_UNCOND("c2 send one wsm pkt fin");
		  break;
	  }
	  case 3:
	  {
//		  NS_LOG_UNCOND("c3 send one wsm pkt");
		  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices_3.Get (a));
//		  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices_3.Get (b));
		  const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
		  Mac48Address bssWildcard = Mac48Address::GetBroadcast ();
		  uint32_t channel = WaveNetDeviceExample::SCH_transform (channel_id);
		  const TxInfo txInfo = TxInfo (channel);
//		  Ptr<Packet> p  = Create<Packet> (100);
		  SeqTsHeader seqTs;
		  uint32_t seq=pkt.time*100;
		  seqTs.SetSeq (seq);
		  seqTs.SetPriority (priority);
		  seqTs.SetSource (a+1);
		  pkt.pkt->AddHeader (seqTs);
		  sender->SendX  (pkt.pkt, bssWildcard, WSMP_PROT_NUMBER, txInfo);
//		  NS_LOG_UNCOND("c3 send one wsm pkt fin");
		  break;
	  }
	  case 4:
	  {
//		  NS_LOG_UNCOND("c4 send one wsm pkt");
		  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices_4.Get (a));
//		  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices_4.Get (b));
		  const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
		  Mac48Address bssWildcard = Mac48Address::GetBroadcast ();
		  uint32_t channel = WaveNetDeviceExample::SCH_transform (channel_id);
		  const TxInfo txInfo = TxInfo (channel);
//		  Ptr<Packet> p  = Create<Packet> (100);
		  SeqTsHeader seqTs;
		  uint32_t seq=pkt.time*100;
		  seqTs.SetSeq (seq);
		  seqTs.SetPriority (priority);
		  seqTs.SetSource (a+1);
		  pkt.pkt->AddHeader (seqTs);
		  sender->SendX  (pkt.pkt, bssWildcard, WSMP_PROT_NUMBER, txInfo);
//		  NS_LOG_UNCOND("c4 send one wsm pkt fin");
		  break;
	  }
	  case 5:
	  {
//		  NS_LOG_UNCOND("c5 send one wsm pkt");
		  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices_5.Get (a));
//		  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices_5.Get (b));
		  const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
		  Mac48Address bssWildcard = Mac48Address::GetBroadcast ();
		  uint32_t channel = WaveNetDeviceExample::SCH_transform (channel_id);
		  const TxInfo txInfo = TxInfo (channel);
//		  Ptr<Packet> p  = Create<Packet> (100);
		  SeqTsHeader seqTs;
		  uint32_t seq=pkt.time*100;
		  seqTs.SetSeq (seq);
		  seqTs.SetPriority (priority);
		  seqTs.SetSource (a+1);
		  pkt.pkt->AddHeader (seqTs);
		  sender->SendX  (pkt.pkt, bssWildcard, WSMP_PROT_NUMBER, txInfo);
//		  NS_LOG_UNCOND("c5 send one wsm pkt fin");
		  break;
	  }
	  case 6:
	  {
//		  NS_LOG_UNCOND("c6 send one wsm pkt");
		  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices_6.Get (a));
//		  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices_6.Get (b));
		  const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
		  Mac48Address bssWildcard = Mac48Address::GetBroadcast ();
		  uint32_t channel = WaveNetDeviceExample::SCH_transform (channel_id);
		  const TxInfo txInfo = TxInfo (channel);
//		  Ptr<Packet> p  = Create<Packet> (100);
		  SeqTsHeader seqTs;
		  uint32_t seq=pkt.time*100;
		  seqTs.SetSeq (seq);
		  seqTs.SetPriority (priority);
		  seqTs.SetSource (a+1);
		  pkt.pkt->AddHeader (seqTs);
		  sender->SendX  (pkt.pkt, bssWildcard, WSMP_PROT_NUMBER, txInfo);
//		  NS_LOG_UNCOND("c6 send one wsm pkt fin");
		  break;
	  }
	  default:break;
  }

}

//main
void
WaveNetDeviceExample::SendWsmpExample ()
{
  CreateWaveNodes ();
  srand((unsigned)time(NULL));
  std::cout <<"sendwsmpexample"<<std::endl;
  Simulator::Schedule (Seconds (1), &WaveNetDeviceExample::channel_count, this);
  Simulator::Schedule (Seconds (stop_time), &WaveNetDeviceExample::report, this);
  Simulator::Stop (Seconds (stop_time));
  Simulator::Run ();
  Simulator::Destroy ();
}

void
WaveNetDeviceExample::SendIpPacket (uint32_t seq, bool ipv6)
{
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));
  const Address dest = receiver->GetAddress ();
  // send IPv4 packet or IPv6 packet
  const static uint16_t IPv4_PROT_NUMBER = 0x0800;
  const static uint16_t IPv6_PROT_NUMBER = 0x86DD;
  uint16_t protocol = ipv6 ? IPv6_PROT_NUMBER : IPv4_PROT_NUMBER;
  Ptr<Packet> p  = Create<Packet> (100);
  SeqTsHeader seqTs;
  seqTs.SetSeq (seq);
  p->AddHeader (seqTs);
  sender->Send (p, dest, protocol);
}

void
WaveNetDeviceExample::SendIpExample ()
{
  CreateWaveNodes ();
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));

  // Alternating access without immediate channel switch
  const SchInfo schInfo = SchInfo (SCH1, false, EXTENDED_ALTERNATING);
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, sender, schInfo);
  // An important point is that the receiver should also be assigned channel
  // access for the same channel to receive packets.
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, receiver, schInfo);

  // both IPv4 and IPv6 packets below will not be inserted to internal queue because of no tx profile registered
  Simulator::Schedule (Seconds (1.0), &WaveNetDeviceExample::SendIpPacket, this, 1, true);
  Simulator::Schedule (Seconds (1.050), &WaveNetDeviceExample::SendIpPacket, this, 2, false);
  //register txprofile
  // IP packets will automatically be sent with txprofile parameter
  const TxProfile txProfile = TxProfile (SCH1);
  Simulator::Schedule (Seconds (2.0), &WaveNetDevice::RegisterTxProfile, sender, txProfile);
  // both IPv4 and IPv6 packet are transmitted successfully
  Simulator::Schedule (Seconds (3.0), &WaveNetDeviceExample::SendIpPacket, this, 3, true);
  Simulator::Schedule (Seconds (3.050), &WaveNetDeviceExample::SendIpPacket, this, 4, false);
  // unregister TxProfile or release channel access
  Simulator::Schedule (Seconds (4.0),&WaveNetDevice::DeleteTxProfile, sender,SCH1);
  Simulator::Schedule (Seconds (4.0),&WaveNetDevice::StopSch, sender,SCH1);
  Simulator::Schedule (Seconds (4.0),&WaveNetDevice::StopSch, receiver, SCH1);
  // these packets will be dropped again because of no channel access assigned and no tx profile registered
  Simulator::Schedule (Seconds (5.0), &WaveNetDeviceExample::SendIpPacket, this, 5, true);
  Simulator::Schedule (Seconds (5.050), &WaveNetDeviceExample::SendIpPacket, this, 6, false);

  Simulator::Stop (Seconds (6.0));
  Simulator::Run ();
  Simulator::Destroy ();
}

bool
WaveNetDeviceExample::ReceiveVsa (Ptr<const Packet> pkt,const Address & address, uint32_t, uint32_t)
{
  std::cout << "receive a VSA management frame: recvTime = " << Now ().GetSeconds () << "s." << std::endl;
  return true;
}

void
WaveNetDeviceExample::SendWsaExample ()
{
  CreateWaveNodes ();
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));

// Alternating access without immediate channel switch for sender and receiver
  const SchInfo schInfo = SchInfo (SCH1, false, EXTENDED_ALTERNATING);
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, sender, schInfo);
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, receiver, schInfo);

// the peer address of VSA is broadcast address, and the repeat rate
// of VsaInfo is 100 per 5s, the VSA frame will be sent repeatedly.
  Ptr<Packet> wsaPacket = Create<Packet> (100);
  Mac48Address dest = Mac48Address::GetBroadcast ();
  const VsaInfo vsaInfo = VsaInfo (dest, OrganizationIdentifier (), 0, wsaPacket, SCH1, 100, VSA_TRANSMIT_IN_BOTHI);
  Simulator::Schedule (Seconds (1.0), &WaveNetDevice::StartVsa, sender, vsaInfo);
  Simulator::Schedule (Seconds (3.0), &WaveNetDevice::StopVsa, sender, SCH1);

// release alternating access
  Simulator::Schedule (Seconds (4.0), &WaveNetDevice::StopSch, sender, SCH1);
  Simulator::Schedule (Seconds (4.0), &WaveNetDevice::StopSch, receiver, SCH1);

// these WSA packets cannot be transmitted because of no channel access assigned
  Simulator::Schedule (Seconds (5.0), &WaveNetDevice::StartVsa, sender, vsaInfo);
  Simulator::Schedule (Seconds (6.0), &WaveNetDevice::StopVsa, sender, SCH1);

  Simulator::Stop (Seconds (6.0));
  Simulator::Run ();
  Simulator::Destroy ();
}

int
main (int argc, char *argv[])
{
  //SeedManager::SetSeed ((unsigned)time(NULL));
  WaveNetDeviceExample example;
  std::cout << "run WAVE WSMP routing service case:" << std::endl;
  example.SendWsmpExample ();
  //std::cout << "run WAVE IP routing service case:" << std::endl;
  //example.SendIpExample ();
  //std::cout << "run WAVE WSA routing service case:" << std::endl;
  //example.SendWsaExample ();
  return 0;
}
