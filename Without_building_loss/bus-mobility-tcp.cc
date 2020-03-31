/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2007 INRIA
 *               2009,2010 Contributors
 *
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
 * Author: Yohanna.WANG <yohanna.wang0924@gmail.com>
*/
#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/udp-client.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/config-store-module.h"

#include "ns3/aodv-helper.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"

#include "ns3/itu-r-1411-los-propagation-loss-model.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/integer.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/wave-helper.h"

#include "ns3/netanim-module.h"
using namespace ns3;
using namespace std;
using namespace dsr;

#ifndef PACK_INF
#define Pks_Payload_Len 1000
#define Pks_Header_Len 10
#define Pks_Len 1010
#define Data_Pks_Header  0x47 //indicate the data packets
#endif
#define Receive_Enough 0x77 // indicate the receiver has got enough data
#define Buffer_Size 15000000 //10MB

#ifndef NULL
#define NULL __null
#endif

#ifndef BOOL
#define BOOL bool
#endif

#define Seed_Buffer_Len 100000
typedef unsigned long DWORD;

NS_LOG_COMPONENT_DEFINE("LT_code_with_UDP");

uint32_t MacTxDropCount, PhyTxDropCount, PhyRxDropCount;
double once_time = 0;
int succnt = 0;
//-------------------------------------------------------------------------
typedef struct StructList
{
  uint32_t data;
  struct StructList *next;
}LIST,*PLIST;

class BP_Index
{
public:
  BP_Index(uint32_t Len);
  ~BP_Index();
  void Ins_BP_Index(uint32_t index,uint32_t data);
  BOOL Del_BP_Index(uint32_t index,uint32_t data);
  uint32_t Get_Del_head(uint32_t index);
public:
  PLIST *m_index;
  uint32_t *m_degree;
};

BP_Index::BP_Index(uint32_t Len)
{
  m_index=new PLIST[Len];
  m_degree=new uint32_t [Len];
  uint32_t i;
  for(i=0;i<Len;i++)
     {
	m_index[i]=NULL;
     }
  for (i=0;i<Len;i++)
     {
	m_degree[i]=0;
     }
}

BP_Index::~BP_Index()
{
  //cout<<"1. delete ~BP_Index"<<endl;
  delete []m_index;
  m_index=NULL;
  //cout<<"2. delete m_degree"<<endl;
  delete []m_degree;
  m_degree=NULL;
}

void BP_Index::Ins_BP_Index(uint32_t index,uint32_t data)
{
  PLIST P_list=m_index[index-1];
  if (P_list==NULL)
  {
     m_index[index-1]=new LIST;
     m_index[index-1]->data=data;
     m_index[index-1]->next=NULL;
  }
  else
  {
     PLIST p_new_list=new LIST;
     p_new_list->data=data;
     p_new_list->next=P_list; 
     m_index[index-1]=p_new_list;		
  }
  m_degree[index-1]++;
}

BOOL BP_Index::Del_BP_Index(uint32_t index,uint32_t data)
{
  PLIST P_list=m_index[index-1];
  PLIST p_back=NULL;
  PLIST p_now=P_list;
  if (p_now==NULL)
     {
	return false;
     }
  uint32_t now_data=p_now->data;
  if (now_data==data)
     {	
      m_index[index-1]=P_list->next;
      //cout<<"3. delete p_now"<<endl;
      delete p_now;
      m_degree[index-1]--;
      return true;
     }
  while ((now_data!=data)&&(p_now!=NULL))
     {
      p_back=p_now;
      p_now=p_now->next;
	if (p_now!=NULL)
	{
	  now_data=p_now->data;
	}	
     }
  if (p_now==NULL)
     {
	return false;
     }
  p_back->next=p_now->next;
  //cout<<"4. delete p_now"<<endl;
  delete p_now;
  m_degree[index-1]--;
  return true;
}

uint32_t BP_Index::Get_Del_head(uint32_t index)
{
  PLIST P_list=m_index[index-1];
  PLIST P_foreward=P_list->next;
  uint32_t data=P_list->data;
  //cout<<"5. delete p_list"<<endl;
  delete P_list;
  m_index[index-1]=P_foreward;
  m_degree[index-1]--;
  return data;
}

class List 
{
public:
  List();
  ~List();
  void Ins_list(uint32_t data);
  uint32_t Get_Del_head(void);
  BOOL Del_list(uint32_t data);
  void Clear(void);
public:
  PLIST m_list;
  uint32_t  m_Len;	
};

List::List()
{
  m_list=NULL;
  m_Len=0;
}

List::~List()
{
  if (m_list)
     {
      //cout<<"6. delete m_list"<<endl;
      delete m_list;
      m_list=NULL;
     }
  m_Len=0;
}

void List::Ins_list(uint32_t data)
{
  if (m_list==NULL)
     {
      m_list=new LIST;
      m_list->data=data;
      m_list->next=NULL;
     }
  else
     {
      PLIST m_new_list=new LIST;
      m_new_list->data=data;
      m_new_list->next=m_list;
      m_list=m_new_list;	
     }
  m_Len++;
}

uint32_t List::Get_Del_head(void)
{
  PLIST P_list=m_list;
  if (P_list!=NULL)
     {
       PLIST P_foreward=P_list->next;
       uint32_t data=P_list->data;
       //cout<<"7. delete p_list"<<endl;
       delete P_list;
       m_list=P_foreward;
       m_Len--;
       return data;
     }
  return 0;
}

BOOL List::Del_list(uint32_t data)
{
  PLIST p_back=NULL;
  PLIST p_now=m_list;
  if (p_now==NULL)
     {
      return false;
     }
  uint32_t now_data=p_now->data;
  if (now_data==data)
     {	
      m_list=m_list->next;
      //cout<<"8. delete p_now"<<endl;
      delete p_now;
      m_Len--;
      return true;
     }
  while ((now_data!=data)&&(p_now!=NULL))
     {
      p_back=p_now;
      p_now=p_now->next;
      if (p_now!=NULL)
         {
           now_data=p_now->data;
         }	
      }
  if (p_now==NULL)
     {
      return false;
     }
  p_back->next=p_now->next;
  //cout<<"9. delete p_now"<<endl;
  delete p_now;
  m_Len--;
  return true;
}

class Seed_Pro 
{
public:
  Seed_Pro(uint32_t len);
  ~Seed_Pro(void);
  void Rand_Seed(void);
  uint32_t Get_Seed(void);
private:
  uint32_t *Seed_Buf;
  uint32_t Seed_Buf_Len;
  uint32_t P_top;
};

Seed_Pro::Seed_Pro(uint32_t len)
{
  Seed_Buf_Len=len;
  P_top=0;
  Seed_Buf=new uint32_t[Seed_Buf_Len];
  Rand_Seed();
}

Seed_Pro::~Seed_Pro(void)
{
  //cout<<"10. delete seed_buf"<<endl;
  delete Seed_Buf;
}

void Seed_Pro::Rand_Seed()
{
  uint32_t i;
  srand( (unsigned)time( NULL )); // use the system time to initialize the random generator
  for (i=0;i<Seed_Buf_Len;i++)
     {
      Seed_Buf[i]=rand()*10000;
     }
}

uint32_t Seed_Pro::Get_Seed() // get random seed
{
  if (P_top==Seed_Buf_Len)
     {
      Rand_Seed();
      P_top=0;
     }
  P_top++;
  return Seed_Buf[P_top-1];
}

uint32_t sum(uint32_t*data,uint32_t len) //calculate sum
{
  uint32_t anssum=0;
  uint32_t i;
  for (i=0;i<len;i++)
     {
      anssum+=data[i];
     }
  return anssum;
}

double* Degree(uint32_t Ori_Pks_Num,double c,double omega)
{
  double * robust_d=new double[Ori_Pks_Num];
  memset(robust_d,0,sizeof(double)*Ori_Pks_Num);
  double *P=new double[Ori_Pks_Num];
  memset(P,0,sizeof(double)*Ori_Pks_Num);
  double *t=new double[Ori_Pks_Num];
  memset(t,0,sizeof(double)*Ori_Pks_Num);
  P[0]=1/(double)Ori_Pks_Num;
  uint32_t i=0;
  for (i=1;i<Ori_Pks_Num;i++)
     {
      P[i]=1/(double)(i*(i+1));
     }
  double s=c*log(Ori_Pks_Num/omega)*sqrt(Ori_Pks_Num);
  for (i=0;i<ceil(Ori_Pks_Num/s)-1;i++)
     {
      t[i]=s/(Ori_Pks_Num*(i+1));
     }
  t[(uint32_t)ceil(Ori_Pks_Num/s)-1]=s/Ori_Pks_Num*log(s/omega);
  double sum=0;
  for (i=0;i<Ori_Pks_Num;i++)
     {
      robust_d[i]=P[i]+t[i];
      sum+=robust_d[i];
     }
  for (i=0;i<Ori_Pks_Num;i++)
     {
      robust_d[i]=robust_d[i]/sum;
      if (i)
	{
	  robust_d[i]=robust_d[i-1]+robust_d[i];
	}		
     }
  //cout<<"11. delete degree p[]"<<endl;
  delete []P;
  //cout<<"12. delete degree t[]"<<endl;
  delete []t;
  return robust_d;
}

BOOL Repeat(uint32_t *Neighbors_I,uint32_t Neighbors_count)
{
  uint32_t i;
  for (i=0;i<Neighbors_count;i++)
     {
      if (Neighbors_I[i]==Neighbors_I[Neighbors_count])
	{
	 return true;
	}	
     }
  return false;
}

//the encoding function
void LT_Encode(char* buf_send,char* buf,DWORD dwFileSize,DWORD LTSendSize)
{
  uint32_t Ori_Pks_Num=ceil((double)dwFileSize/(double)Pks_Payload_Len);
  uint32_t Enc_Pks_Num=ceil((double)LTSendSize/(double)Pks_Len);
  //cout<<"OUT1:: Ori_Pks_Num = "<<Ori_Pks_Num<<" Enc_Pks_Num = "<<Enc_Pks_Num<<endl;

  class Seed_Pro seed(Seed_Buffer_Len);
  uint32_t i;
  double *degree=Degree(Ori_Pks_Num,0.01,0.5);
  char *Pks_I=new char [Pks_Len];// a new packet
  char *Pks_read_I=new char[Pks_Payload_Len];
  for (i=0;i<Enc_Pks_Num;i++)
     {
      *(uint16_t*)Pks_I = Data_Pks_Header;
      *(uint32_t*)(Pks_I+2)=dwFileSize;//file length
      uint32_t Seed_Once=seed.Get_Seed();
      *(uint32_t*)(Pks_I+6)=Seed_Once;
      srand( Seed_Once);
      double Rand_Num=(double)rand()/((double)RAND_MAX+1);
      uint32_t degree_I=0;
      uint32_t j=0;
      for (j=0;j<Ori_Pks_Num;j++)
	{
	  if (Rand_Num<degree[j])
	     {
		degree_I=j+1;
		break;
	     }
	  degree_I=Ori_Pks_Num;
	}
      uint32_t *Neighbors_I=new uint32_t [degree_I];
      memset(Neighbors_I,0,sizeof(uint32_t)*degree_I);
      for (uint32_t Neighbors_count=0;Neighbors_count<degree_I;Neighbors_count++)
	{			
	 Neighbors_I[Neighbors_count]=floor(Ori_Pks_Num*(double)rand()/((double)RAND_MAX+1))+1;
	 while(Repeat(Neighbors_I,Neighbors_count))
	      {	
		Neighbors_I[Neighbors_count]=floor(Ori_Pks_Num*(double)rand()/((double)RAND_MAX+1))+1;
	      }
	memcpy(Pks_read_I,buf+(Neighbors_I[Neighbors_count]-1)*Pks_Payload_Len,Pks_Payload_Len);
	if (!Neighbors_count)
	      {
		memcpy(Pks_I+Pks_Header_Len,Pks_read_I,sizeof(char)*Pks_Payload_Len);
	      }
	else
	      {
		for (uint32_t l=0;l<Pks_Payload_Len;l++)
		     {
			Pks_I[Pks_Header_Len+l]^=Pks_read_I[l];
		     }
	      }
			
	}
	 memcpy(buf_send+Pks_Len*i,Pks_I,Pks_Len);
         //cout<<"13. delete Neighbors_I"<<endl;
	 //delete []Neighbors_I;
	}
  //cout<<"14. delete Pks_I"<<endl;
  //delete []Pks_I;
  //cout<<"15. delete Pks_read_I"<<endl;
  //delete []Pks_read_I;
  //delete []Neighbors_I;	
}

BOOL LT_Decode(char* buf,char* buf_Receive,DWORD m_dwFileSize,DWORD m_LTSendSize)
{
  char *Pks_D_I=new char [Pks_Len];
  memcpy(Pks_D_I,buf_Receive,Pks_Len);
  uint32_t filelen = *(uint32_t*)(Pks_D_I+2);
  uint32_t Ori_Pks_Num = ceil((double)filelen/(double)Pks_Payload_Len);
  uint32_t Enc_Pks_Num = m_LTSendSize/Pks_Len;
  char *Pks_buffer=new char[Pks_Payload_Len*Enc_Pks_Num];
  memset(Pks_buffer,0,sizeof(char)*Pks_Payload_Len*Enc_Pks_Num);

  uint32_t *Ori_bp_process=new uint32_t[Ori_Pks_Num];
  memset(Ori_bp_process,0,sizeof(uint32_t)*Ori_Pks_Num);
  BP_Index Enc_bp_index (Enc_Pks_Num);
  BP_Index Ori_bp_index (Ori_Pks_Num);
  List Enc_bp_ripple;

  char *Temp_pk1=new char[Pks_Payload_Len];
  char *Temp_pk2=new char[Pks_Payload_Len];
  uint32_t seed=0;
  uint32_t Ori_index=0;
  uint32_t Enc_index=0;

  double *degree=Degree(Ori_Pks_Num,0.01,0.5);
  uint32_t i;
  for (i=0;i<Enc_Pks_Num;i++)
     {	
      memcpy(Pks_D_I,buf_Receive+Pks_Len*i,Pks_Len);
      seed=*(uint32_t*)(Pks_D_I+6);
      memcpy(Pks_buffer+Pks_Payload_Len*i,Pks_D_I+Pks_Header_Len,Pks_Payload_Len);
      srand(seed);
      double Rand_D_Num=(double)rand()/((double)RAND_MAX+1);
      uint32_t degree_D_I=0;
      uint32_t j=0;
      for (j=0;j<Ori_Pks_Num;j++)
	{
	  if (Rand_D_Num<degree[j])
	     {
		degree_D_I=j+1;
		break;
	     }
	  degree_D_I=Ori_Pks_Num;
	}
      uint32_t *Neighbors_I=new uint32_t [degree_D_I];
      memset(Neighbors_I,0,sizeof(uint32_t)*degree_D_I);
      for (uint32_t Neighbors_count=0;Neighbors_count<degree_D_I;Neighbors_count++)
	{			
	  Neighbors_I[Neighbors_count]=floor(Ori_Pks_Num*(double)rand()/((double)RAND_MAX+1))+1;
	  while(Repeat(Neighbors_I,Neighbors_count))
	     {	
		Neighbors_I[Neighbors_count]=floor(Ori_Pks_Num*(double)rand()/((double)RAND_MAX+1))+1;
	     }	
	}
      for (uint32_t count_I=0;count_I<degree_D_I;count_I++)
	{
	  Ori_index=Neighbors_I[count_I];
	  Enc_bp_index.Ins_BP_Index((i+1),Ori_index);
	  Ori_bp_index.Ins_BP_Index(Ori_index,i+1);
	}
     //cout<<"16. delete Neighbors_I"<<endl;
     //delete []Neighbors_I;	
     }
   
  for (i=0;i<Enc_Pks_Num;i++)
     {
      if (Enc_bp_index.m_degree[i]==1)
	{
	  Enc_bp_ripple.Ins_list(i+1);
	}
     }
  while (Enc_bp_ripple.m_Len)
     {
      Enc_index=Enc_bp_ripple.Get_Del_head();
      Ori_index=Enc_bp_index.Get_Del_head(Enc_index);				
      Ori_bp_index.Del_BP_Index(Ori_index,Enc_index);				

      memcpy(buf+Pks_Payload_Len*(Ori_index-1),Pks_buffer+Pks_Payload_Len*(Enc_index-1),Pks_Payload_Len);
      Ori_bp_process[Ori_index-1]=1;
      if (sum(Ori_bp_process,Ori_Pks_Num)==Ori_Pks_Num)
	 {
          cout<<"FINAL: all packets decoded here"<<endl;
	  break;
	 }
      else if(sum(Ori_bp_process, Ori_Pks_Num) < Ori_Pks_Num)
         {
          //cout<<"OUT: Decode not finished && Summary = "<<sum(Ori_bp_process, Ori_Pks_Num)<<std::endl;
         }  
      if (Ori_bp_index.m_degree[Ori_index-1])
	 {
	  while(Ori_bp_index.m_degree[Ori_index-1])
               {
		Enc_index=Ori_bp_index.Get_Del_head(Ori_index);
		Enc_bp_index.Del_BP_Index(Enc_index,Ori_index);
		memcpy(Temp_pk1,Pks_buffer+Pks_Payload_Len*(Enc_index-1),Pks_Payload_Len);
		memcpy(Temp_pk2,buf+Pks_Payload_Len*(Ori_index-1),Pks_Payload_Len);
		for (uint32_t l=0;l<Pks_Payload_Len;l++)
		   {
	             Temp_pk1[l]^=Temp_pk2[l];
	           }	
		memcpy(Pks_buffer+Pks_Payload_Len*(Enc_index-1),Temp_pk1,Pks_Payload_Len);			
		if (Enc_bp_index.m_degree[Enc_index-1]==1)
		   {
		     Enc_bp_ripple.Ins_list(Enc_index);
		   }
		if (Enc_bp_index.m_degree[Enc_index-1]==0)
	           {
	             Enc_bp_ripple.Del_list(Enc_index);
	           }
		}
					
	 }
				
     }		
  if (sum(Ori_bp_process,Ori_Pks_Num)==Ori_Pks_Num)
     {
      //cout<<"18. delete Pks_D_I"<<endl;
      //delete []Pks_D_I;
      //cout<<"19. delete Pks_buffer"<<endl;
      //delete []Pks_buffer;
      //cout<<"20. delete Ori_bp_process"<<endl;
      //delete []Ori_bp_process;
      //cout<<"21. delete Temp_PK1"<<endl;
      //delete []Temp_pk1;
      //cout<<"22. delete Temp_PK2"<<endl;
      //delete []Temp_pk2;
      cout<<"OUT: decode finished successfully"<<endl;
      return true;
     }
      //cout<<"23. delete Pks_D_I"<<endl;
      //delete []Pks_D_I;
      //cout<<"24. delete Pks_buffer"<<endl;
      //delete []Pks_buffer;
      //cout<<"25. delete Ori_bp_process"<<endl;
      //delete []Ori_bp_process;
      //cout<<"26. delete Temp_PK1"<<endl;
      //delete []Temp_pk1;
      //cout<<"27. delete Temp_PK2"<<endl;
      //delete []Temp_pk2;
      //std::cout<<"OUT: Decode failed "<<std::endl;
      return false;
}
//-------------------------------------------------------------------------
//Client Application
class MyAPPC: public Application
{
public:
  MyAPPC();
  virtual ~MyAPPC();
  void Setup(Ptr<Socket> socket, Address address, uint32_t packetSize,
			DataRate dataRate);

private:
  virtual void StartApplication(void);
  virtual void StopApplication(void);

  void ScheduleTx(void);
  void SendPacket(void);

  Ptr<Socket> m_socket;
  Address m_peer;
  uint32_t m_packetSize;
  uint32_t m_nPackets;
  DataRate m_dataRate;
  EventId m_sendEvent;
  bool m_running;
  uint32_t m_packetsSent;

  char * m_sendBuffer;
  long m_sendPos;
  long m_sendSize;
};

MyAPPC::MyAPPC() :
        m_socket(0), 
        m_peer(), 
        m_packetSize(0), 
        m_nPackets(0), 
        m_dataRate(0), 
        m_sendEvent(), 
        m_running(false), 
        m_packetsSent(0) 
{
}

MyAPPC::~MyAPPC() 
{
  m_socket = 0;
  m_packetSize = 0;
  m_nPackets = 0;
  m_packetsSent = 0;
  m_sendPos = 0;
  m_sendSize = 0;
  delete []m_sendBuffer;
}

void MyAPPC::Setup(Ptr<Socket> socket, Address address, 
		 uint32_t packetSize, DataRate dataRate) 
{
  //cout<<"c. MyAppc setup"<<endl;
  m_socket = socket;
  m_peer = address;
  m_packetSize = packetSize;
  m_dataRate = dataRate;

  std::ifstream fsend("200_200_3.txt", std::ifstream::binary);//*****************************************************************************
  fsend.seekg(0, std::ifstream::end);
  uint32_t flen = fsend.tellg();
  char * filereadbuff = new char[flen];
  fsend.seekg(0, std::ifstream::beg);
  fsend.read(filereadbuff, flen);
  fsend.close();

  m_sendBuffer = new char[flen];
  memset(m_sendBuffer,0,flen);

  m_sendSize = flen;
  m_sendPos = 0;

  //flen = 0;
  /*
  cout<<__PRETTY_FUNCTION__<<" : Setup  "<<
            InetSocketAddress::ConvertFrom(m_peer).GetIpv4()<<" ["<<
            InetSocketAddress::ConvertFrom(m_peer) <<"]--"<<endl;
  */
}

void MyAPPC::StartApplication(void) 
{
  m_running = true;
  m_packetsSent = 0;
  m_socket->Bind();
  m_socket->Connect(m_peer);

  cout<<"Sender starts sending at: "<< Simulator::Now ().GetSeconds()<<endl;
  SendPacket();//start to send data
}

void MyAPPC::StopApplication(void) 
{
  //cout<<"e. stop application"<<endl;
  m_running = false;
  if (m_sendEvent.IsRunning()) 
     {
      Simulator::Cancel(m_sendEvent);
     }
  if (m_socket) 
     {
      m_socket->Close();
     }
}

void MyAPPC::SendPacket(void) 
{
  //cout<<"f. send packet"<<endl;
  Ptr<Packet> packet = Create<Packet>(
		(const uint8_t *) m_sendBuffer + m_sendPos, m_packetSize);
  m_socket->Send(packet);

  if (m_sendPos < m_sendSize) 
     {
      ScheduleTx();
      m_sendPos += m_packetSize;
     } 
  else
     {
      cout<<"Sender stops sending at: "<< Simulator::Now ().GetSeconds()<<endl; //
      m_running = false;
     }      
}

void MyAPPC::ScheduleTx(void) 
{
  if (m_running) 
     {
      /*
      Time tNext(Seconds(m_packetSize * 128 //8
	 / static_cast<double>(m_dataRate.GetBitRate())));           
      m_sendEvent = Simulator::Schedule(tNext, &MyAPPC::SendPacket, this);
      */
      m_sendEvent = Simulator::Schedule(Seconds(0.1), &MyAPPC::SendPacket, this);      
     }
}
//------------------------------------------------------------------
/*
static void CourseChange (std::ostream *os, std::string foo, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition (); 
  Vector vel = mobility->GetVelocity (); 

  *os << Simulator::Now () << " POS: x=" << pos.x << ", y=" << pos.y
      << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
      << ", z=" << vel.z << std::endl;
}
*/
/*
static void CwndChange(uint32_t oldCwnd, uint32_t newCwnd) 
{
  NS_LOG_INFO (Simulator::Now ().GetSeconds () << "\t" << newCwnd);
  once_time = Simulator::Now ().GetSeconds ();
}

static void ReceivePkt(Ptr<const Packet> p){
	NS_LOG_INFO ("Receive at "<<Simulator::Now().GetSeconds());
}


static void RxDrop(Ptr<const Packet> p) 
{
  NS_LOG_INFO ("RxDrop at " << Simulator::Now ().GetSeconds ());
}
*/
//------------------------------------------------------------------
class VANET
{
public: 
  VANET();
  void Simulate();
  void CommandSetup(int argc, char **argv);
  static NodeContainer m_vanetTxNodes;
  static NetDeviceContainer m_vanetTxDevices;
  static Ipv4InterfaceContainer m_interfaces;

private:
  void SetupLogFile ();
  void SetupLogging ();
  void ConfigureDefaults();
  void SetupMobilityNodes();
  void SetupAdhocDevices();
  void SetupScenario();

  int  m_verbose;
  double m_duration;
  uint32_t m_nNodes;
  uint32_t m_mobility;
  uint32_t m_protocol;
  uint32_t m_lossModel;
  uint32_t m_fading;
  uint32_t m_scenario;

  bool verbose;  
  bool ascii;
  bool pcap;

  std::string m_lossModelName;
  std::string m_traceFile;
  std::string m_logFile;
  std::string m_phyMode;
  std::string m_phyModeB;
  std::string m_protocolName;
  std::ofstream m_os;

};

NodeContainer VANET::m_vanetTxNodes;
NetDeviceContainer VANET::m_vanetTxDevices;
Ipv4InterfaceContainer VANET::m_interfaces;

VANET::VANET()
  : m_verbose(0),
    m_duration(319.90),
    m_nNodes(60),
    m_mobility(1),
    m_protocol(2),
    m_lossModel(3),
    m_fading(0),
    m_scenario(1),
    verbose(false),
    ascii(false),
    pcap(false),
    m_lossModelName (""),
    m_traceFile("./scratch/109_bus_mobility.tcl"),
    m_logFile("109-bus_log.log"),
    m_phyMode("OfdmRate6MbpsBW10MHz"),
    m_phyModeB("DsssRate11Mbps"),
    m_protocolName ("protocol")
{
}

void VANET::CommandSetup(int argc, char **argv)
{
  CommandLine cmd;
  cmd.AddValue ("m_traceFile", "Ns2 movement trace file(109_bus_mobility)", m_traceFile);
  cmd.AddValue ("m_logFile", "Log file=109-bus_log.log", m_logFile);
  cmd.AddValue ("m_phyMode", "VANET phy mode(OfdmRate6MbpsBW10MHz)", m_phyMode);
  cmd.AddValue ("phyModeB", "Phy mode 802.11b", m_phyModeB);
  cmd.AddValue ("lossModel", "1=Friis;2=ItuR1411Los;3=TwoRayGround;4=LogDistance", m_lossModel);
  cmd.AddValue ("fading", "0=None;1=Nakagami;(buildings=1 overrides)", m_fading);
  cmd.AddValue ("m_nNodes", "Number of nodes(60)", m_nNodes);
  cmd.AddValue ("m_duration", "Duration of Simulation(319.90)", m_duration);
  cmd.AddValue ("mobility", "1=BUS,2=Vehicles,3=BUS+Vehicles", m_mobility);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("ascii", "Turn on ASCII trace function", ascii);
  cmd.AddValue ("pcap", "Turn on PCAP trace function", pcap);
  cmd.AddValue ("verbose", "0=quiet;1=verbose", m_verbose);
  cmd.AddValue ("protocol", "1=OLSR;2=AODV;3=DSDV;4=DSR", m_protocol);
  cmd.AddValue ("m_scenario", "VANET scenario for selection", m_scenario);
  cmd.Parse (argc,argv);
}

void VANET::SetupLogFile()
{
  m_os.open (m_logFile.c_str ());
}

void VANET::SetupLogging()
{
  //LogComponentEnable ("Ns2MobilityHelper",LOG_LEVEL_DEBUG);
}

void VANET::ConfigureDefaults()
{
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (m_phyModeB));
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (m_phyMode));
  //Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                   //MakeBoundCallback (&CourseChange, &m_os));
}

void VANET::SetupMobilityNodes()
{
  if (m_mobility == 1)
    {
    Ns2MobilityHelper ns2 = Ns2MobilityHelper (m_traceFile);
    VANET::m_vanetTxNodes.Create (m_nNodes);
    ns2.Install (); 
    }
  else if (m_mobility == 2) 
    {
    //here is left for the mobility of VEHICLES(925-2_Bus_mobility.tcl)
    //Manhattan Models && Random Waypoint Models prepared here too.
    }
  else if (m_mobility ==3)
    {
    //Here is left for the mobility of BUS+Vehicles(925-3_bus_mobility.tcl)
    }
}

void VANET::SetupAdhocDevices()
{
  if (m_lossModel == 1)
    {
      m_lossModelName = "ns3::FriisPropagationLossModel";
    }
  else if (m_lossModel == 2)
    {
      m_lossModelName = "ns3::ItuR1411LosPropagationLossModel";
    }
  else if (m_lossModel == 3)
    {
      m_lossModelName = "ns3::TwoRayGroundPropagationLossModel";
    }
  else if (m_lossModel == 4)
    {
      m_lossModelName = "ns3::LogDistancePropagationLossModel";
    }
  else
    {
      NS_LOG_ERROR ("Invalid propagation loss model specified.Values must be [1-4],where                  1=Friis;2=ItuR1411Los;3=TwoRayGround;4=LogDistance");
    }
  // 802.11p 5.9 GHz
  double freq = 5.9e9;

  // Setup propagation models
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  if (m_lossModel == 3)
    {
      // two-ray requires antenna height (else defaults to Friss)
      wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq), "HeightAboveZ", DoubleValue (1.5));
    }
  else
    {
      wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq));
    }

  // Propagation loss models are additive.
  if (m_fading != 0)
    {
      // if no obstacle model, then use Nakagami fading if requested
      wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
    }
  // the channel
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);

  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default();
  if (m_verbose)
    {
      wifi80211p.EnableLogComponents (); 
    }
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (m_phyModeB),
                                "ControlMode",StringValue (m_phyModeB));

  // Setup 802.11p stuff
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (m_phyMode),
                                      "ControlMode",StringValue (m_phyMode));

  m_vanetTxDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, m_vanetTxNodes);

  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");

  if(ascii)
  {
    AsciiTraceHelper ascii;
    wifiPhy.EnableAsciiAll(ascii.CreateFileStream("1-4-vanet.tr"));
  }
  if(pcap)
  {
    wifiPhy.EnablePcap ("1-4vanet", m_vanetTxDevices);
  }

  AodvHelper aodv;
  OlsrHelper olsr;
  DsdvHelper dsdv;
  DsrHelper dsr;
  DsrMainHelper dsrMain;

  Ipv4ListRoutingHelper list;
  InternetStackHelper internet;

  switch (m_protocol)
    {
    case 1:
      list.Add (olsr, 100);
      m_protocolName = "OLSR";
      break;
    case 2:
      list.Add (aodv, 100);
      m_protocolName = "AODV";
      break;
    case 3:
      list.Add (dsdv, 100);
      m_protocolName = "DSDV";
      break;
    case 4:
      m_protocolName = "DSR";
      break;
    default:
      NS_FATAL_ERROR ("No such protocol:" << m_protocol);
    }

   if (m_protocol < 4)
    {
      internet.SetRoutingHelper (list);
      internet.Install (VANET::m_vanetTxNodes);
    }
  else if (m_protocol == 4)
    {
      internet.Install (VANET::m_vanetTxNodes);
      dsrMain.Install (dsr, VANET::m_vanetTxNodes);
    }

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  m_interfaces = ipv4.Assign (m_vanetTxDevices);           

}

void VANET::SetupScenario()
{
  if (m_scenario == 1)
    {
      // Realistic vehicle traces build from SUMO+OSM
      m_traceFile = "scrach/109_bus_mobility.tcl";
      m_logFile = "109bus_mobility.log";
      m_mobility = 1;
      m_nNodes = 60;
      m_duration = 319.90;
    }
  else if (m_scenario == 2)
    {
      m_traceFile = "827_bus_mobility.tcl";
      m_logFile = "827bus_mobility.log";
      m_mobility = 1;
      m_nNodes = 29;
      m_duration = 999.90;      
    }
}
void VANET::Simulate()
{
  SetupLogFile ();
  SetupLogging ();
  ConfigureDefaults();
  SetupMobilityNodes();
  SetupAdhocDevices();
  SetupScenario();
}
//------------------------------------------------------------------
void MacTxDrop(Ptr<const Packet> p)
{
  NS_LOG_INFO("Packet Drop");
  MacTxDropCount++;
}

void PrintDrop()
{
  std::cout << "At time: " << Simulator::Now().GetSeconds() << "\t" <<
          "MaxTxDropCount : " << MacTxDropCount << "\t"<< 
          "PhyTxDropCount : " << PhyTxDropCount << "\t" << 
          "PhyRxDropCount : " << PhyRxDropCount << "\n";
  Simulator::Schedule(Seconds(5.0), &PrintDrop);
}

void PhyTxDrop(Ptr<const Packet> p)
{
  NS_LOG_INFO("Packet Drop");
  PhyTxDropCount++;
}
void PhyRxDrop(Ptr<const Packet> p)
{
  NS_LOG_INFO("Packet Drop");
  PhyRxDropCount++;
}

//------------------------------------------------------------------
int main (int argc, char *argv[])
{
//*******************************
  std::cout << "Usage of  " << argv[0] << " :\n\n"
            "./waf --run '/scratch/1-4-bus-mobility --PrintHelp' --vis \n" 
            "This is a test of TCP+Data dissemination+VANET+SUMO Trace \n"
            "Designed by Yohanna.WANG (finished in 2017-1-6 11:23 am-) \n"
            "60 bus nodes with duration of 319.90s(HK MongKok District)\n";
//*******************************
  //double total = 0;
  uint32_t m_sinkPort = 8080;
  VANET vanet;
  vanet.CommandSetup(argc, argv);
  vanet.Simulate();
//--------------------------------------------------------------------------------------------------
  // the sever part
  for(int i =1; i<30; i++)
  {
  Address sinkAddress(InetSocketAddress(VANET::m_interfaces.GetAddress(0), m_sinkPort));
  PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort));
  ApplicationContainer sinkApps = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (0));
  sinkApps.Start (Seconds (1.));
  sinkApps.Stop (Seconds (40.));
 
  Ptr<Socket> ns3TcpSocket = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (i), TcpSocketFactory::GetTypeId ());
  //ns3TcpSocket->TraceConnectWithoutContext ("CongestionWindow", MakeCallback (&CwndChange));

  Ptr<MyAPPC> app = CreateObject<MyAPPC> ();
  app->Setup (ns3TcpSocket, sinkAddress, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (i)->AddApplication (app);
  app->SetStartTime (Seconds (0.));
  app->SetStopTime (Seconds (40.));

  //VANET::m_vanetTxDevices.Get(1)->TraceConnectWithoutContext("PhyRxDrop",MakeCallback(&RxDrop));
  //VANET::m_vanetTxDevices.Get(1)->TraceConnectWithoutContext("ReceivePkt", MakeCallback(&ReceivePkt));
  }
//--------------------------------------------------------------------------------------------------------
  // Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  // Trace Collisions
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop", MakeCallback(&MacTxDrop));
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&PhyRxDrop));
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback(&PhyTxDrop));

  Simulator::Schedule(Seconds(5.0), &PrintDrop);

  Simulator::Stop (Seconds (40.));
  Simulator::Run ();
  PrintDrop();

  // Print per flow statistics
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin (); iter != stats.end (); ++iter)
    {
	  Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first);

      if (( t.sourceAddress == Ipv4Address("10.1.1.1")  && t.destinationAddress == Ipv4Address("10.1.1.2"))
    	 || (t.sourceAddress == Ipv4Address("10.1.1.1") && t.destinationAddress == Ipv4Address("10.1.1.3")))
        {
    	  NS_LOG_UNCOND("Flow ID: " << iter->first << " Src Addr " << t.sourceAddress << " Dst Addr " << t.destinationAddress);
    	  NS_LOG_UNCOND("Tx Packets = " << iter->second.txPackets);
    	  NS_LOG_UNCOND("Rx Packets = " << iter->second.rxPackets);
    	  NS_LOG_UNCOND("Throughput: " << iter->second.rxBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds()) / 1024  << " Kbps");
        }
    }
  monitor->SerializeToXmlFile("lab.flowmon", true, true);

  Simulator::Destroy ();

  return 0;
}
