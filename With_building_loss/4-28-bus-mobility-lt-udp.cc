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
#include "ns3/gpsr-module.h"

#include "ns3/itu-r-1411-los-propagation-loss-model.h"
#include <ns3/buildings-helper.h>
#include <ns3/hybrid-buildings-propagation-loss-model.h>

#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/integer.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/netanim-module.h"

#include "ns3/flow-monitor-module.h"

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
double total = 0;
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
  cout<<"OUT1:: Ori_Pks_Num = "<<Ori_Pks_Num<<" Enc_Pks_Num = "<<Enc_Pks_Num<<endl;

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
          cout<<"OUT: Decode not finished && Summary = "<<sum(Ori_bp_process, Ori_Pks_Num)<<std::endl;
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

  void ReceivePacket(Ptr<Socket>);
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
  //cout<<"b. ~MyAppc "<<endl;
  m_socket = 0;
  //cout<<"28. delete []m_sendBuffer"<<endl;
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
  m_sendPos = 0;
  /*
  cout<<__PRETTY_FUNCTION__<<" : Setup  "<<
            InetSocketAddress::ConvertFrom(m_peer).GetIpv4()<<" ["<<
            InetSocketAddress::ConvertFrom(m_peer) <<"]--"<<endl;
  */
}

void MyAPPC::StartApplication(void) 
{
  //cout<<"d. startApplication "<<endl;	
  m_running = true;
  m_packetsSent = 0;
  m_socket->Bind();
  m_socket->Connect(m_peer);

  std::ifstream fsend("Vehicle.mp4", std::ifstream::binary);//*****************************************************************************
  fsend.seekg(0, std::ifstream::end);
  uint32_t flen = fsend.tellg();
  char * filereadbuff = new char[flen];
  fsend.seekg(0, std::ifstream::beg);
  fsend.read(filereadbuff, flen);
  fsend.close();

  m_sendSize = 4 * flen;// this is the maximal data that will be sent
  m_sendBuffer = new char[m_sendSize];
  memset(m_sendBuffer, 0, m_sendSize);
  
  LT_Encode(m_sendBuffer, filereadbuff, flen, m_sendSize);
  m_sendPos = 0;
  NS_LOG_FUNCTION(this << m_socket);

  m_socket->SetRecvCallback(MakeCallback(&MyAPPC::ReceivePacket, this));//receive data from receiver

  //cout<<"Sender starts sending at: "<< Simulator::Now ().GetSeconds()<<endl;
  SendPacket();//start to send data
  // cout<<"29. delete []fileReadBuffer"<<endl;
  delete []filereadbuff;
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
  SeqTsHeader seqTs;
  seqTs.SetSeq(m_packetsSent);
  Ptr<Packet> packet = Create<Packet>(
		(const uint8_t *) m_sendBuffer + m_sendPos, m_packetSize);
  packet->AddHeader(seqTs);
  m_socket->Send(packet);

  if (m_sendPos < m_sendSize) 
     {
      ScheduleTx();
      m_sendPos += m_packetSize;
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
      m_sendEvent = Simulator::Schedule(Seconds(0.08), &MyAPPC::SendPacket, this);
      
     }
}

void MyAPPC::ReceivePacket(Ptr<Socket> socket) 
{

  NS_LOG_FUNCTION (this << socket);
  Ptr<Packet> packet;
  Address from;

  cout<<__PRETTY_FUNCTION__<<" : Receive  "<<
            InetSocketAddress::ConvertFrom(from).GetIpv4()<<" ["<<
            InetSocketAddress::ConvertFrom(from) <<"]--"<<endl;

  while ((packet = socket->RecvFrom(from)))
	{
          NS_LOG_UNCOND("Received one packet here.");
	  if (packet->GetSize() > 0) 
             {
	        SeqTsHeader seqTs;
		packet->RemoveHeader(seqTs);
		uint16_t* header = new uint16_t;
		packet->CopyData((uint8_t*) header,2);
		if ( (* header) == Receive_Enough)//if the receiver has got sufficient data, the sender should stop sending
		   {
		     m_running = false;
		     std::cout<<"Sender stops sending at: "<< Simulator::Now ().GetSeconds()<<std::endl;
		   }
	     }

	}
}

//Sever Application------------------------------------------------------------------
class MyAPPS: public Application {
public:
  static TypeId GetTypeId (void);
  MyAPPS();
  virtual ~MyAPPS();
  void Setup(Address adrr, uint16_t port, uint32_t packetSize, DataRate dataRate);
  uint32_t GetLost(void) const;
  uint32_t GetReceived(void) const;
  uint16_t GetPacketWindowSize() const;
  void SetPacketWindowSize(uint16_t size);

protected:
  virtual void DoDispose(void);

private:
  virtual void StartApplication(void);
  virtual void StopApplicatioin(void);
  void ReceivePacket(Ptr<Socket>);
  void ScheduleRx();//Address
  void SendPacket(Address);//Address

  Ptr<Socket> m_socket;
  Ptr<Socket> m_socket6;
  uint16_t m_port;
  Address m_localadrr;
  uint32_t m_packetSize;
  uint32_t m_packetsSent;
  uint32_t m_nPackets;
  DataRate m_dataRate;
  EventId m_receiveEvent;
  PacketLossCounter m_lossCounter;
  bool m_OK;
  bool m_running;
  uint32_t m_packetsReceive;
  char * m_sendBuffer;
  long m_sendPos;
  long m_sendSize;
  char * m_receiveBuffer;//we will initialize the size to be 10MB
  long m_receivePos;
  long m_receiveSize;
  long m_decodethreshold;// when the received data is above this value, start to try decoding
};

TypeId MyAPPS::GetTypeId(void) 
{
  //cout<<"i. MyApps get typeid"<<endl;
  static TypeId tid = TypeId("ns3::MyAPPS").SetParent<Application>().AddConstructor<
				MyAPPS>().AddAttribute("Port",
				"Port on which we listen for incoming packets.",
				UintegerValue(100),
				MakeUintegerAccessor(&MyAPPS::m_port),
				MakeUintegerChecker<uint16_t>()).AddAttribute(
				"PacketWindowSize",
				"The size of the window used to compute the packet loss. This value should be a multiple of 8.",
				UintegerValue(32),
				MakeUintegerAccessor(&MyAPPS::GetPacketWindowSize,
				&MyAPPS::SetPacketWindowSize),
				MakeUintegerChecker<uint16_t>(8, 256));
  return tid;
}

MyAPPS::MyAPPS() :
        m_socket(0), 
        m_packetSize(0), 
        m_nPackets(0), 
        m_dataRate(0), 
        m_receiveEvent(),
        m_lossCounter(0), 
        m_running(false), 
        m_packetsReceive(0)
{
}

MyAPPS::~MyAPPS() 
{
  //cout<<"k. MyApps release"<<endl;
  m_socket = 0;
  delete []m_receiveBuffer;
}

void MyAPPS::Setup(Address adrr, uint16_t port, uint32_t packetSize, DataRate dataRate) 
{
  //cout<<"l. MyApps setup "<<endl;
  m_localadrr = adrr;
  m_port = port;
  m_packetSize = packetSize;
  m_dataRate = dataRate;
  m_receiveBuffer = new char[Buffer_Size];
  m_OK = false;
  m_receivePos = 0;
}

uint16_t MyAPPS::GetPacketWindowSize() const 
{
  cout<<"m. MyApps GetPacketWindowSize()"<<endl;
  NS_LOG_FUNCTION (this);
  return m_lossCounter.GetBitMapSize();
}

void MyAPPS::SetPacketWindowSize(uint16_t size) 
{
  //cout<<"n. MyApps SetPacketWindowSize()"<<endl;
  NS_LOG_FUNCTION (this << size);
  m_lossCounter.SetBitMapSize(size);
}

uint32_t MyAPPS::GetLost(void) const 
{
  cout<<"o. MyApps GetLost(void)"<<endl;
  NS_LOG_FUNCTION (this);
  return m_lossCounter.GetLost();
}

uint32_t MyAPPS::GetReceived(void) const 
{
  cout<<"p. MyApps GetReceived(void)"<<endl;
  NS_LOG_FUNCTION (this);
  return m_packetsReceive;
}

void MyAPPS::DoDispose(void) 
{
  //cout<<"q. MyApps DoDispose(void)"<<endl;
  NS_LOG_FUNCTION (this);
  Application::DoDispose();
}

void MyAPPS::StartApplication() 
{
  //cout<<"r. MyApps StartApplication()"<<endl;
  NS_LOG_FUNCTION (this);

  if (m_socket == 0) 
     {
      TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
      m_socket = Socket::CreateSocket(GetNode(), tid);
      m_socket->Bind(m_localadrr);
     }
  m_socket->SetRecvCallback(MakeCallback(&MyAPPS::ReceivePacket, this));//receive data call back
}

void MyAPPS::StopApplicatioin() 
{
  //scout<<"s. MyApps StopApplicatioin()"<<endl;
  NS_LOG_FUNCTION (this);

  if (m_socket != 0) 
     {
      m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket> >());
     }
}

void MyAPPS::ReceivePacket(Ptr<Socket> socket) 
{
  //cout<<"t. Continue P"<<endl;
  NS_LOG_FUNCTION (this << socket);
  Ptr<Packet> packet;
  Address from;

  while ((packet = socket->RecvFrom(from))) 
     {
      if (packet->GetSize() > 0) 
        {
	SeqTsHeader seqTs;
	packet->RemoveHeader(seqTs);
	uint8_t* header = new uint8_t [6];
	uint32_t filelen = 0;
	uint16_t type = 0;
	packet->CopyData((uint8_t*) header,6);
	type = * (uint16_t*)header;//identifier
	filelen = * (uint32_t*)(header + 2);
	m_decodethreshold = filelen*1.01;
	if (type == Data_Pks_Header && !m_OK ) // check if it is data packet and if we have decoded the original file
	  {
	     packet->CopyData((uint8_t*) m_receiveBuffer + m_receivePos, m_packetSize);
     	     m_receivePos += m_packetSize;
             //cout<<"1. Receive:  "<< m_packetsReceive << " at: " << Simulator::Now ().GetSeconds() <<endl; 

	     if (m_receivePos > m_decodethreshold && m_packetsReceive % 100 == 0 )
		{
	          char * buf = new char [filelen];
                  std::cout<<"1. Receive: "<<m_packetsReceive<<" at: "
                           <<Simulator::Now().GetSeconds()<<"m_receivePos= "
                           <<m_receivePos<<" "<<"decode Threshold = "<<m_decodethreshold<<std::endl;

		  if (LT_Decode(buf,m_receiveBuffer,filelen,m_receivePos))
		     {
		       std::ofstream freceived("vehicle_new.mp4",std::ofstream::binary);
		       freceived.write(buf, filelen);
	               freceived.close();
		       std::cout<< "Receiver's Decoding succeeds at: "<< Simulator::Now ().GetSeconds()<<std::endl;
                       cout<<"OUT:. filelen = "<<filelen <<endl;   

                       once_time = Simulator::Now ().GetSeconds();
                       cout<<"once_time after decode = "<<once_time<<endl;
                       succnt += 1;
                       cout<<"Meaning of succnt = "<<succnt<<endl;			    

                       m_OK = true;
		       m_sendBuffer = new char [2];

                       cout<<"set m_ok=true && m_sendbuffer =2"<<endl;
		       *(uint16_t *) m_sendBuffer = Receive_Enough;//stop message identifier
                       
	               m_sendPos = 0;
		       m_packetSize = 2;
		       SendPacket(from);//SendPacket(from) send the stop message to the sender*/
		       m_packetsSent ++;
		     }
		  else
		     {
		       std::cout<<"Receiver's Decoding fails Once!"<<std::endl;
	             }
                  //cout<<"32. delete []buf"<<endl;
                  delete []buf;

		}
	}
	m_packetsReceive++; 
			
	uint32_t currentSequenceNumber = seqTs.GetSeq();
        if (InetSocketAddress::IsMatchingType(from)) 
          {
	    NS_LOG_INFO("TraceDelay: RX " << packet->GetSize () <<
	    " bytes from "<< InetSocketAddress::ConvertFrom (from).GetIpv4 () <<
	    " Sequence Number: " << currentSequenceNumber <<
	    " Uid: " << packet->GetUid () <<
	    " TXtime: " << seqTs.GetTs () <<
	    " RXtime: " << Simulator::Now () <<
	    " Delay: " << Simulator::Now () - seqTs.GetTs ());
          }

	    //m_lossCounter.NotifyReceived(currentSequenceNumber);

		}
	}
}
void MyAPPS::SendPacket(Address dest)//
{
  //cout<<"u. SendPackets"<<endl;
  SeqTsHeader seqTs;
  seqTs.SetSeq(m_packetsSent);
  Ptr<Packet> packet = Create<Packet>(
	     (const uint8_t *) m_sendBuffer + m_sendPos, m_packetSize);
  cout<<"m_send Pos && m_packetSize after decode finished "<<m_sendPos<<"  "<<m_packetSize<<endl;
  packet->AddHeader(seqTs);

  NS_LOG_FUNCTION(this << m_socket);
  int res = m_socket->SendTo(packet, 0, dest); //SendTo(packet,0,dest)

  if(res < 0)
     {
      cout<<__PRETTY_FUNCTION__<<" Error to send: returned "<<res<<endl;
     }
  else
     {    
      cout<<__PRETTY_FUNCTION__<<" : Sent "<<m_packetSize<< " bytes to "<<
            InetSocketAddress::ConvertFrom(dest).GetIpv4()<<" ["<<
            InetSocketAddress::ConvertFrom(dest) <<"]--' "<<m_sendBuffer<<
            " ' "<<endl;    
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

static void ReceivePkt(Ptr<const Packet> p){
	NS_LOG_INFO ("Receive at "<<Simulator::Now().GetSeconds());
}

static void CwndChange(uint32_t oldCwnd, uint32_t newCwnd) {
	NS_LOG_INFO (Simulator::Now ().GetSeconds () << "\t" << newCwnd);
        
}

static void RxDrop(Ptr<const Packet> p) 
{
	NS_LOG_INFO ("RxDrop at " << Simulator::Now ().GetSeconds ());
}
*/
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
  Simulator::Schedule(Seconds(50.0), &PrintDrop);
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
  double m_TxRange;
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
    m_duration(1199.90),  //319.9
    m_TxRange(300.0),
    m_nNodes(199), //60
    m_mobility(1),
    m_protocol(2),
    m_lossModel(3),
    m_fading(0),
    m_scenario(1),
    verbose(false),
    ascii(false),
    pcap(false),
    m_lossModelName (""),
    m_traceFile("./scratch/528-400-vehicle_trace.tcl"),//109_bus_mobility.tcl
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
  cmd.AddValue ("m_nNodes", "Number of nodes(110)", m_nNodes);
  cmd.AddValue ("m_duration", "Duration of Simulation(1199.90)", m_duration);
  cmd.AddValue ("mobility", "1=BUS,2=Vehicles,3=BUS+Vehicles", m_mobility);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("ascii", "Turn on ASCII trace function", ascii);
  cmd.AddValue ("pcap", "Turn on PCAP trace function", pcap);
  cmd.AddValue ("verbose", "0=quiet;1=verbose", m_verbose);
  cmd.AddValue ("protocol", "1=OLSR;2=AODV;3=DSDV;4=GPSR;5=DSR", m_protocol);
  cmd.AddValue ("m_scenario", "VANET scenario for selection", m_scenario);
  cmd.AddValue ("m_TxRange", "Building Propagation loss range", m_TxRange);
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
  else if (m_lossModel == 5)
    {
      m_lossModelName = "ns3::HybridBuildingsPropagationLossModel";
    }
  else
    {
      NS_LOG_ERROR ("Invalid propagation loss model specified.Values must be [1-4],where                  1=Friis;2=ItuR1411Los;3=TwoRayGround;4=LogDistance;5=HybridBuilding;");
    }

  // 802.11p 5.9 GHz
  double freq = 5.9e9;

  // Setup propagation models
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel","MaxRange", DoubleValue (m_TxRange));  //300.0 //4-27

  if (m_lossModel == 3)
    {
      // two-ray requires antenna height (else defaults to Friss)
      wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq), "HeightAboveZ", DoubleValue (1.5));
    }
//---------------------------------------------------------------------------------------//
  if (m_lossModel == 5)
    {
//----------------------Mong Kok building information added here------------------------//

  //Building 1 (building id = 316844253 + 316843182)
  Ptr<Building> b1 = CreateObject <Building> ();
  b1->SetBoundaries (Box (1037.0, 1052.0, 311.0, 357.0, 5.0, 20.0));
  b1->SetBuildingType (Building::Residential);
  b1->SetExtWallsType (Building::ConcreteWithWindows);
  b1->SetNFloors (6);
  b1->SetNRoomsX (2);
  b1->SetNRoomsY (1); 
  //Building 2 (building id = 316843427 + 316843256)
  Ptr<Building> b2 = CreateObject <Building> ();
  b2->SetBoundaries (Box (1053.0, 1070.0, 315.0, 357.0, 5.0, 20.0));
  b2->SetBuildingType (Building::Residential);
  b2->SetExtWallsType (Building::ConcreteWithWindows);
  b2->SetNFloors (6);
  b2->SetNRoomsX (2);
  b2->SetNRoomsY (1); 
  //Building 3 (building id = 316843286 + 316844381)
  Ptr<Building> b3 = CreateObject <Building> ();
  b3->SetBoundaries (Box (1085.0, 1100.0, 320.0, 365.0, 5.0, 20.0));
  b3->SetBuildingType (Building::Residential);
  b3->SetExtWallsType (Building::ConcreteWithWindows);
  b3->SetNFloors (6);
  b3->SetNRoomsX (2);
  b3->SetNRoomsY (1); 
  //Building 4 (building id = 316843081 + 316843769)
  Ptr<Building> b4 = CreateObject <Building> ();
  b4->SetBoundaries (Box (1109.0, 1130.0, 325.0, 369.0, 5.0, 20.0));
  b4->SetBuildingType (Building::Residential);
  b4->SetExtWallsType (Building::ConcreteWithWindows);
  b4->SetNFloors (6);
  b4->SetNRoomsX (2);
  b4->SetNRoomsY (1); 
  //Building 5 (building id = 316844028 + 316843578)
  Ptr<Building> b5 = CreateObject <Building> ();
  b5->SetBoundaries (Box (1142.0, 1164.0, 337.0, 375.0, 5.0, 20.0));
  b5->SetBuildingType (Building::Commercial);
  b5->SetExtWallsType (Building::ConcreteWithWindows);
  b5->SetNFloors (6);
  b5->SetNRoomsX (2);
  b5->SetNRoomsY (1); 
  //Building 6 (building id = 316843768 + 316843245)
  Ptr<Building> b6 = CreateObject <Building> ();
  b6->SetBoundaries (Box (1169.0, 1187.0, 335.0, 357.0, 5.0, 20.0));
  b6->SetBuildingType (Building::Residential);
  b6->SetExtWallsType (Building::ConcreteWithWindows);
  b6->SetNFloors (6);
  b6->SetNRoomsX (2);
  b6->SetNRoomsY (1); 
  //Building 7 (building id = 316843379)
  Ptr<Building> b7 = CreateObject <Building> ();
  b7->SetBoundaries (Box (1053.0, 1070.0, 315.0, 357.0, 5.0, 2.0));
  b7->SetBuildingType (Building::Commercial);
  b7->SetExtWallsType (Building::Wood);
  b7->SetNFloors (1);
  b7->SetNRoomsX (2);
  b7->SetNRoomsY (1);
  //Building 8 (building id = 316843254 + 316843611)
  Ptr<Building> b8 = CreateObject <Building> ();
  b8->SetBoundaries (Box (1045.0, 1051.0, 215.0, 300.0, 0.0, 20.0));
  b8->SetBuildingType (Building::Residential);
  b8->SetExtWallsType (Building::ConcreteWithWindows);
  b8->SetNFloors (6);
  b8->SetNRoomsX (2);
  b8->SetNRoomsY (1);
  //Building 9 (building id = 316843227 + 316844003)
  Ptr<Building> b9 = CreateObject <Building> ();
  b9->SetBoundaries (Box (1060.0, 1077.0, 220.0, 302.0, 0.0, 20.0));
  b9->SetBuildingType (Building::Residential);
  b9->SetExtWallsType (Building::ConcreteWithWindows);
  b9->SetNFloors (6);
  b9->SetNRoomsX (2);
  b9->SetNRoomsY (1);
  //Building 10 (building id = 316843660 + 316844490)
  Ptr<Building> b10 = CreateObject <Building> ();
  b10->SetBoundaries (Box (1092.0, 1108.0, 225.0, 311.0, 0.0, 20.0));
  b10->SetBuildingType (Building::Residential);
  b10->SetExtWallsType (Building::ConcreteWithWindows);
  b10->SetNFloors (6);
  b10->SetNRoomsX (2);
  b10->SetNRoomsY (1);
  //Building 11 (building id = 316843167 + 316844255 + 316843767)
  Ptr<Building> b11 = CreateObject <Building> ();
  b11->SetBoundaries (Box (1116.0, 1126.0, 229.0, 311.0, 0.0, 20.0));
  b11->SetBuildingType (Building::Residential);
  b11->SetExtWallsType (Building::ConcreteWithWindows);
  b11->SetNFloors (6);
  b11->SetNRoomsX (2);
  b11->SetNRoomsY (1);
  //Building 12 (building id = 316844256 + 316843462 + 316843067)
  Ptr<Building> b12 = CreateObject <Building> ();
  b12->SetBoundaries (Box (1151.0, 1195.0, 235.0, 320.0, 0.0, 20.0));
  b12->SetBuildingType (Building::Residential);
  b12->SetExtWallsType (Building::ConcreteWithWindows);
  b12->SetNFloors (6);
  b12->SetNRoomsX (2);
  b12->SetNRoomsY (1);
  //Building 13 (building id = 316843066 + 316843772)
  Ptr<Building> b13 = CreateObject <Building> ();
  b13->SetBoundaries (Box (1230.0, 1265.0, 251.0, 331.0, 0.0, 20.0));
  b13->SetBuildingType (Building::Residential);
  b13->SetExtWallsType (Building::ConcreteWithWindows);
  b13->SetNFloors (6);
  b13->SetNRoomsX (2);
  b13->SetNRoomsY (1);
  //Building 14 (building id = 316843282)
  Ptr<Building> b14 = CreateObject <Building> ();
  b14->SetBoundaries (Box (1079.0, 1093.0, 175.0, 194.0, 0.0, 20.0));
  b14->SetBuildingType (Building::Residential);
  b14->SetExtWallsType (Building::ConcreteWithWindows);
  b14->SetNFloors (6);
  b14->SetNRoomsX (2);
  b14->SetNRoomsY (1);
  //Building 15 (building id = 185564130)
  Ptr<Building> b15 = CreateObject <Building> ();
  b15->SetBoundaries (Box (1113.0, 1165.0, 18.0, 202.0, 0.0, 20.0));
  b15->SetBuildingType (Building::Commercial);
  b15->SetExtWallsType (Building::ConcreteWithWindows);
  b15->SetNFloors (6);
  b15->SetNRoomsX (2);
  b15->SetNRoomsY (1);
  //Building 16 (building id = 316843271)
  Ptr<Building> b16 = CreateObject <Building> ();
  b16->SetBoundaries (Box (1171.0, 1190.0, 187.0, 209.0, 0.0, 20.0));
  b16->SetBuildingType (Building::Commercial);
  b16->SetExtWallsType (Building::ConcreteWithWindows);
  b16->SetNFloors (6);
  b16->SetNRoomsX (2);
  b16->SetNRoomsY (1);
  //Building 17 (building id = 316843191)
  Ptr<Building> b17 = CreateObject <Building> ();
  b17->SetBoundaries (Box (1193.0, 1223.0, 153.0, 213.0, 0.0, 20.0));
  b17->SetBuildingType (Building::Commercial);
  b17->SetExtWallsType (Building::ConcreteWithWindows);
  b17->SetNFloors (6);
  b17->SetNRoomsX (2);
  b17->SetNRoomsY (1);
  //Building 18 (building id = 316843587 + 316844359)
  Ptr<Building> b18 = CreateObject <Building> ();
  b18->SetBoundaries (Box (1248.0, 1288.0, 183.0, 225.0, 0.0, 20.0));
  b18->SetBuildingType (Building::Commercial);
  b18->SetExtWallsType (Building::ConcreteWithWindows);
  b18->SetNFloors (6);
  b18->SetNRoomsX (2);
  b18->SetNRoomsY (1);


      //Parameters required in Hybrid building propagation loss model
      wifiChannel.AddPropagationLoss (m_lossModelName,
                                      "Frequency", DoubleValue (freq),
                                      "Environment", StringValue("Urban"),
                                      "CitySize", StringValue("Small"),
                                      "ShadowSigmaOutdoor", DoubleValue (7.0), //7,8,5,5
                                      "ShadowSigmaIndoor", DoubleValue (8.0),  
                                      "ShadowSigmaExtWalls", DoubleValue (5.0), 
                                      "InternalWallLoss", DoubleValue (5.0)); 
       BuildingsHelper bb;
       bb.Install (m_vanetTxNodes);
       BuildingsHelper::MakeMobilityModelConsistent();
    }
//---------------------------------------------------------------------------------------//
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
  //WaveHelper waveHelper = WaveHelper::Default ();
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

  //WifiMacHelper wifiMac;
  //wifiMac.SetType ("ns3::AdhocWifiMac");

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
  GpsrHelper gpsr;
  //gpsr.Install ();

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
      list.Add(gpsr, 100);
      m_protocolName = "GPSR";
      break;
    case 5:
      m_protocolName = "DSR";
      break;
    default:
      NS_FATAL_ERROR ("No such protocol:" << m_protocol);
    }

   if (m_protocol < 5)
    {
      internet.SetRoutingHelper (list);
      internet.Install (VANET::m_vanetTxNodes);
    }
  else if (m_protocol == 5)
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
      m_traceFile = "scrach/528-400-vehicle_trace.tcl";
      m_logFile = "109bus_mobility.log";
      m_mobility = 1;
      m_nNodes = 199;
      m_duration = 1199.90;
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

void SetupApplication()
{

//-----------------------------------------------------------------------------------------------------------//1
  uint32_t m_sinkPort1 = 8051;
  //receiver
  Address sinkAddress(InetSocketAddress (VANET::m_interfaces.GetAddress(16), m_sinkPort1)); //i
  Ptr<MyAPPS> appS = CreateObject<MyAPPS>();
  appS->Setup(sinkAddress, m_sinkPort1, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(16)->AddApplication(appS);    //i
  appS->SetStartTime(Seconds(25.));
  appS->SetStopTime(Seconds(200.));
  //sender
  Ptr<Socket> ns3UdpSocket = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(1),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC = CreateObject<MyAPPC>();
  appC->Setup(ns3UdpSocket, sinkAddress, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(1)->AddApplication(appC);
  appC->SetStartTime(Seconds(25.));
  appC->SetStopTime(Seconds(200.));

//-----------------------------------------------------------------------------------------------------------///2
  uint32_t m_sinkPort2 = 8052;
   //receiver
  Address sinkAddress2(InetSocketAddress (VANET::m_interfaces.GetAddress(26), m_sinkPort2)); 
  Ptr<MyAPPS> appS2 = CreateObject<MyAPPS>();
  appS2->Setup(sinkAddress2, m_sinkPort2, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(26)->AddApplication(appS2);    
  appS2->SetStartTime(Seconds(50.));
  appS2->SetStopTime(Seconds(200.));
  Ptr<Socket> ns3UdpSocket2 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(1),UdpSocketFactory::GetTypeId());
  //sender
  Ptr<MyAPPC> appC2 = CreateObject<MyAPPC>();
  appC2->Setup(ns3UdpSocket2, sinkAddress2, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(1)->AddApplication(appC2);
  appC2->SetStartTime(Seconds(51.));
  appC2->SetStopTime(Seconds(200.));

 //-----------------------------------------------------------------------------------------------------------///3
  uint32_t m_sinkPort3 = 8053;
  //receiver
  Address sinkAddress3(InetSocketAddress (VANET::m_interfaces.GetAddress(1), m_sinkPort3)); 
  Ptr<MyAPPS> appS3 = CreateObject<MyAPPS>();
  appS3->Setup(sinkAddress3, m_sinkPort3, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(1)->AddApplication(appS3);    
  appS3->SetStartTime(Seconds(120.));
  appS3->SetStopTime(Seconds(200.));
  //sender
  Ptr<Socket> ns3UdpSocket3 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(30),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC3 = CreateObject<MyAPPC>();
  appC3->Setup(ns3UdpSocket3, sinkAddress3, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(30)->AddApplication(appC3);
  appC3->SetStartTime(Seconds(121.));
  appC3->SetStopTime(Seconds(200.));

//-----------------------------------------------------------------------------------------------------------///4
  uint32_t m_sinkPort4 = 8054;
  //receiver
  Address sinkAddress4(InetSocketAddress (VANET::m_interfaces.GetAddress(20), m_sinkPort4)); 
  Ptr<MyAPPS> appS4 = CreateObject<MyAPPS>();
  appS4->Setup(sinkAddress4, m_sinkPort4, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(20)->AddApplication(appS4);    
  appS4->SetStartTime(Seconds(50.));
  appS4->SetStopTime(Seconds(250.));
  //sender
  Ptr<Socket> ns3UdpSocket4 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(2),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC4 = CreateObject<MyAPPC>();
  appC4->Setup(ns3UdpSocket4, sinkAddress4, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(2)->AddApplication(appC4);
  appC4->SetStartTime(Seconds(51.));
  appC4->SetStopTime(Seconds(250.));

//-----------------------------------------------------------------------------------------------------------///5

  uint32_t m_sinkPort5 = 8055;
  //receiver
  Address sinkAddress5(InetSocketAddress (VANET::m_interfaces.GetAddress(24), m_sinkPort5)); 
  Ptr<MyAPPS> appS5 = CreateObject<MyAPPS>();
  appS5->Setup(sinkAddress5, m_sinkPort5, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(24)->AddApplication(appS5);    
  appS5->SetStartTime(Seconds(100.));
  appS5->SetStopTime(Seconds(250.));
  //sender
  Ptr<Socket> ns3UdpSocket5 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(5),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC5 = CreateObject<MyAPPC>();
  appC5->Setup(ns3UdpSocket5, sinkAddress5, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(5)->AddApplication(appC5);
  appC5->SetStartTime(Seconds(101.));
  appC5->SetStopTime(Seconds(250.));

//-----------------------------------------------------------------------------------------------------------///6
  uint32_t m_sinkPort6 = 8056;
  //receiver
  Address sinkAddress6(InetSocketAddress (VANET::m_interfaces.GetAddress(20), m_sinkPort6)); 
  Ptr<MyAPPS> appS6 = CreateObject<MyAPPS>();
  appS6->Setup(sinkAddress6, m_sinkPort6, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(20)->AddApplication(appS6);    
  appS6->SetStartTime(Seconds(200.));
  appS6->SetStopTime(Seconds(350.));
  //sender
  Ptr<Socket> ns3UdpSocket6 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(13),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC6 = CreateObject<MyAPPC>();
  appC6->Setup(ns3UdpSocket6, sinkAddress6, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(13)->AddApplication(appC6);
  appC6->SetStartTime(Seconds(201.));
  appC6->SetStopTime(Seconds(350.));

//-----------------------------------------------------------------------------------------------------------///7
  uint32_t m_sinkPort7 = 8057;
  //receiver
  Address sinkAddress7(InetSocketAddress (VANET::m_interfaces.GetAddress(31), m_sinkPort7)); 
  Ptr<MyAPPS> appS7 = CreateObject<MyAPPS>();
  appS7->Setup(sinkAddress3, m_sinkPort7, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(31)->AddApplication(appS7);    
  appS7->SetStartTime(Seconds(100.));
  appS7->SetStopTime(Seconds(250.));
  //sender
  Ptr<Socket> ns3UdpSocket7 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(3),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC7 = CreateObject<MyAPPC>();
  appC7->Setup(ns3UdpSocket7, sinkAddress7, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(3)->AddApplication(appC7);
  appC7->SetStartTime(Seconds(101.));
  appC7->SetStopTime(Seconds(250.));

//-----------------------------------------------------------------------------------------------------------///8
  uint32_t m_sinkPort8 = 8058;
  //receiver
  Address sinkAddress8(InetSocketAddress (VANET::m_interfaces.GetAddress(44), m_sinkPort8)); 
  Ptr<MyAPPS> appS8 = CreateObject<MyAPPS>();
  appS8->Setup(sinkAddress8, m_sinkPort8, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(44)->AddApplication(appS8);    
  appS8->SetStartTime(Seconds(300.));
  appS8->SetStopTime(Seconds(400.));
  //sender
  Ptr<Socket> ns3UdpSocket8 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(19),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC8 = CreateObject<MyAPPC>();
  appC8->Setup(ns3UdpSocket8, sinkAddress8, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(19)->AddApplication(appC8);
  appC8->SetStartTime(Seconds(301.));
  appC8->SetStopTime(Seconds(400.));

//-----------------------------------------------------------------------------------------------------------///9
  uint32_t m_sinkPort9 = 8059;
  //receiver
  Address sinkAddress9(InetSocketAddress (VANET::m_interfaces.GetAddress(54), m_sinkPort9)); 
  Ptr<MyAPPS> appS9 = CreateObject<MyAPPS>();
  appS9->Setup(sinkAddress9, m_sinkPort9, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(54)->AddApplication(appS9);    
  appS9->SetStartTime(Seconds(400.));
  appS9->SetStopTime(Seconds(500.));
  //sender
  Ptr<Socket> ns3UdpSocket9 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(40),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC9 = CreateObject<MyAPPC>();
  appC9->Setup(ns3UdpSocket9, sinkAddress9, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(40)->AddApplication(appC9);
  appC9->SetStartTime(Seconds(401.));
  appC9->SetStopTime(Seconds(500.));

//-----------------------------------------------------------------------------------------------------------///10
  uint32_t m_sinkPort10 = 8060;
  //receiver
  Address sinkAddress10(InetSocketAddress (VANET::m_interfaces.GetAddress(61), m_sinkPort10)); 
  Ptr<MyAPPS> appS10 = CreateObject<MyAPPS>();
  appS10->Setup(sinkAddress10, m_sinkPort10, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(61)->AddApplication(appS10);    
  appS10->SetStartTime(Seconds(300.));
  appS10->SetStopTime(Seconds(400.));
  //sender
  Ptr<Socket> ns3UdpSocket10 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(34),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC10 = CreateObject<MyAPPC>();
  appC10->Setup(ns3UdpSocket10, sinkAddress10, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(34)->AddApplication(appC10);
  appC10->SetStartTime(Seconds(301.));
  appC10->SetStopTime(Seconds(400.));

//-----------------------------------------------------------------------------------------------------------///11
  uint32_t m_sinkPort11 = 8061;
  //receiver
  Address sinkAddress11(InetSocketAddress (VANET::m_interfaces.GetAddress(70), m_sinkPort11)); 
  Ptr<MyAPPS> appS11 = CreateObject<MyAPPS>();
  appS11->Setup(sinkAddress11, m_sinkPort11, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(70)->AddApplication(appS11);    
  appS11->SetStartTime(Seconds(500.));
  appS11->SetStopTime(Seconds(600.));
  //sender
  Ptr<Socket> ns3UdpSocket11 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(56),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC11 = CreateObject<MyAPPC>();
  appC11->Setup(ns3UdpSocket11, sinkAddress11, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(56)->AddApplication(appC11);
  appC11->SetStartTime(Seconds(501.));
  appC11->SetStopTime(Seconds(600.));

//-----------------------------------------------------------------------------------------------------------///12
  uint32_t m_sinkPort12 = 8062;
  //receiver
  Address sinkAddress12(InetSocketAddress (VANET::m_interfaces.GetAddress(81), m_sinkPort12)); 
  Ptr<MyAPPS> appS12 = CreateObject<MyAPPS>();
  appS12->Setup(sinkAddress12, m_sinkPort12, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(81)->AddApplication(appS12);    
  appS12->SetStartTime(Seconds(300.));
  appS12->SetStopTime(Seconds(500.));
  //sender
  Ptr<Socket> ns3UdpSocket12 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(68),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC12 = CreateObject<MyAPPC>();
  appC12->Setup(ns3UdpSocket12, sinkAddress12, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(68)->AddApplication(appC12);
  appC12->SetStartTime(Seconds(301.));
  appC12->SetStopTime(Seconds(500.));

//-----------------------------------------------------------------------------------------------------------///13
  uint32_t m_sinkPort13 = 8063;
  //receiver
  Address sinkAddress13(InetSocketAddress (VANET::m_interfaces.GetAddress(75), m_sinkPort13)); 
  Ptr<MyAPPS> appS13 = CreateObject<MyAPPS>();
  appS13->Setup(sinkAddress13, m_sinkPort13, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(75)->AddApplication(appS13);    
  appS13->SetStartTime(Seconds(400.));
  appS13->SetStopTime(Seconds(500.));
  //sender
  Ptr<Socket> ns3UdpSocket13 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(50),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC13 = CreateObject<MyAPPC>();
  appC13->Setup(ns3UdpSocket13, sinkAddress13, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(50)->AddApplication(appC13);
  appC13->SetStartTime(Seconds(401.));
  appC13->SetStopTime(Seconds(500.));

//-----------------------------------------------------------------------------------------------------------///14
  uint32_t m_sinkPort14 = 8064;
  //receiver
  Address sinkAddress14(InetSocketAddress (VANET::m_interfaces.GetAddress(107), m_sinkPort14)); 
  Ptr<MyAPPS> appS14 = CreateObject<MyAPPS>();
  appS14->Setup(sinkAddress14, m_sinkPort14, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(107)->AddApplication(appS14);    
  appS14->SetStartTime(Seconds(500.));
  appS14->SetStopTime(Seconds(700.));
  //sender
  Ptr<Socket> ns3UdpSocket14 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(77),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC14 = CreateObject<MyAPPC>();
  appC14->Setup(ns3UdpSocket14, sinkAddress14, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(77)->AddApplication(appC14);
  appC14->SetStartTime(Seconds(501.));
  appC14->SetStopTime(Seconds(700.));

//-----------------------------------------------------------------------------------------------------------///15
  uint32_t m_sinkPort15 = 8065;
  //receiver
  Address sinkAddress15(InetSocketAddress (VANET::m_interfaces.GetAddress(97), m_sinkPort15)); 
  Ptr<MyAPPS> appS15 = CreateObject<MyAPPS>();
  appS15->Setup(sinkAddress15, m_sinkPort15, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(97)->AddApplication(appS15);    
  appS15->SetStartTime(Seconds(450.));
  appS15->SetStopTime(Seconds(550.));
  //sender
  Ptr<Socket> ns3UdpSocket15 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(92),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC15 = CreateObject<MyAPPC>();
  appC15->Setup(ns3UdpSocket15, sinkAddress15, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(92)->AddApplication(appC15);
  appC15->SetStartTime(Seconds(451.));
  appC15->SetStopTime(Seconds(550.));

//-----------------------------------------------------------------------------------------------------------///16
  uint32_t m_sinkPort16 = 8066;
  //receiver
  Address sinkAddress16(InetSocketAddress (VANET::m_interfaces.GetAddress(111), m_sinkPort16)); 
  Ptr<MyAPPS> appS16 = CreateObject<MyAPPS>();
  appS16->Setup(sinkAddress16, m_sinkPort16, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(111)->AddApplication(appS16);    
  appS16->SetStartTime(Seconds(550.));
  appS16->SetStopTime(Seconds(700.));
  //sender
  Ptr<Socket> ns3UdpSocket16 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(102),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC16 = CreateObject<MyAPPC>();
  appC16->Setup(ns3UdpSocket16, sinkAddress16, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(102)->AddApplication(appC16);
  appC16->SetStartTime(Seconds(551.));
  appC16->SetStopTime(Seconds(700.));

//-----------------------------------------------------------------------------------------------------------///17
  uint32_t m_sinkPort17 = 8067;
  //receiver
  Address sinkAddress17(InetSocketAddress (VANET::m_interfaces.GetAddress(128), m_sinkPort17)); 
  Ptr<MyAPPS> appS17 = CreateObject<MyAPPS>();
  appS17->Setup(sinkAddress17, m_sinkPort17, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(128)->AddApplication(appS17);    
  appS17->SetStartTime(Seconds(700.));
  appS17->SetStopTime(Seconds(900.));
  //sender
  Ptr<Socket> ns3UdpSocket17 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(115),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC17 = CreateObject<MyAPPC>();
  appC17->Setup(ns3UdpSocket17, sinkAddress17, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(115)->AddApplication(appC17);
  appC17->SetStartTime(Seconds(701.));
  appC17->SetStopTime(Seconds(900.));

//-----------------------------------------------------------------------------------------------------------///18
  uint32_t m_sinkPort18 = 8068;
  //receiver
  Address sinkAddress18(InetSocketAddress (VANET::m_interfaces.GetAddress(112), m_sinkPort18)); 
  Ptr<MyAPPS> appS18 = CreateObject<MyAPPS>();
  appS18->Setup(sinkAddress18, m_sinkPort18, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(112)->AddApplication(appS18);    
  appS18->SetStartTime(Seconds(650.));
  appS18->SetStopTime(Seconds(800.));
  //sender
  Ptr<Socket> ns3UdpSocket18 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(123),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC18 = CreateObject<MyAPPC>();
  appC18->Setup(ns3UdpSocket18, sinkAddress18, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(123)->AddApplication(appC18);
  appC18->SetStartTime(Seconds(651.));
  appC18->SetStopTime(Seconds(800.));

//-----------------------------------------------------------------------------------------------------------///19
  uint32_t m_sinkPort19 = 8069;
  //receiver
  Address sinkAddress19(InetSocketAddress (VANET::m_interfaces.GetAddress(118), m_sinkPort19)); 
  Ptr<MyAPPS> appS19 = CreateObject<MyAPPS>();
  appS19->Setup(sinkAddress19, m_sinkPort19, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(118)->AddApplication(appS19);    
  appS19->SetStartTime(Seconds(800.));
  appS19->SetStopTime(Seconds(900.));
  //sender
  Ptr<Socket> ns3UdpSocket19 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(106),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC19 = CreateObject<MyAPPC>();
  appC19->Setup(ns3UdpSocket19, sinkAddress19, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(106)->AddApplication(appC19);
  appC19->SetStartTime(Seconds(851.));
  appC19->SetStopTime(Seconds(900.));

//-----------------------------------------------------------------------------------------------------------///20
  uint32_t m_sinkPort20 = 8070;
  //receiver
  Address sinkAddress20(InetSocketAddress (VANET::m_interfaces.GetAddress(145), m_sinkPort20)); 
  Ptr<MyAPPS> appS20 = CreateObject<MyAPPS>();
  appS20->Setup(sinkAddress20, m_sinkPort20, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(145)->AddApplication(appS20);    
  appS20->SetStartTime(Seconds(700.));
  appS20->SetStopTime(Seconds(800.));
  //sender
  Ptr<Socket> ns3UdpSocket20 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(135),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC20 = CreateObject<MyAPPC>();
  appC20->Setup(ns3UdpSocket20, sinkAddress20, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(135)->AddApplication(appC20);
  appC20->SetStartTime(Seconds(701.));
  appC20->SetStopTime(Seconds(800.));
//-----------------------------------------------------------------------------------------------------------///21
  uint32_t m_sinkPort21 = 8071;
  //receiver
  Address sinkAddress21(InetSocketAddress (VANET::m_interfaces.GetAddress(18), m_sinkPort21)); 
  Ptr<MyAPPS> appS21 = CreateObject<MyAPPS>();
  appS21->Setup(sinkAddress21, m_sinkPort21, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(18)->AddApplication(appS21);    
  appS21->SetStartTime(Seconds(250.));
  appS21->SetStopTime(Seconds(350.));
  //sender
  Ptr<Socket> ns3UdpSocket21 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(12),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC21 = CreateObject<MyAPPC>();
  appC21->Setup(ns3UdpSocket21, sinkAddress21, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(12)->AddApplication(appC21);
  appC21->SetStartTime(Seconds(251.));
  appC21->SetStopTime(Seconds(350.));

//-----------------------------------------------------------------------------------------------------------///22
  uint32_t m_sinkPort22 = 8072;
  //receiver
  Address sinkAddress22(InetSocketAddress (VANET::m_interfaces.GetAddress(43), m_sinkPort22)); 
  Ptr<MyAPPS> appS22 = CreateObject<MyAPPS>();
  appS22->Setup(sinkAddress22, m_sinkPort22, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(43)->AddApplication(appS22);    
  appS22->SetStartTime(Seconds(400.));
  appS22->SetStopTime(Seconds(500.));
  //sender
  Ptr<Socket> ns3UdpSocket22 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(39),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC22 = CreateObject<MyAPPC>();
  appC22->Setup(ns3UdpSocket22, sinkAddress22, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(39)->AddApplication(appC22);
  appC22->SetStartTime(Seconds(401.));
  appC22->SetStopTime(Seconds(500.));

//-----------------------------------------------------------------------------------------------------------///23
  uint32_t m_sinkPort23 = 8073;
  //receiver
  Address sinkAddress23(InetSocketAddress (VANET::m_interfaces.GetAddress(71), m_sinkPort23)); 
  Ptr<MyAPPS> appS23 = CreateObject<MyAPPS>();
  appS23->Setup(sinkAddress23, m_sinkPort23, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(71)->AddApplication(appS23);    
  appS23->SetStartTime(Seconds(450.));
  appS23->SetStopTime(Seconds(550.));
  //sender
  Ptr<Socket> ns3UdpSocket23 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(58),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC23 = CreateObject<MyAPPC>();
  appC23->Setup(ns3UdpSocket23, sinkAddress23, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(58)->AddApplication(appC23);
  appC23->SetStartTime(Seconds(451.));
  appC23->SetStopTime(Seconds(550.));

//-----------------------------------------------------------------------------------------------------------///24
  uint32_t m_sinkPort24 = 8074;
  //receiver
  Address sinkAddress24(InetSocketAddress (VANET::m_interfaces.GetAddress(121), m_sinkPort24)); 
  Ptr<MyAPPS> appS24 = CreateObject<MyAPPS>();
  appS24->Setup(sinkAddress24, m_sinkPort24, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(121)->AddApplication(appS24);    
  appS24->SetStartTime(Seconds(600.));
  appS24->SetStopTime(Seconds(700.));
  //sender
  Ptr<Socket> ns3UdpSocket24 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(93),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC24 = CreateObject<MyAPPC>();
  appC24->Setup(ns3UdpSocket24, sinkAddress24, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(93)->AddApplication(appC24);
  appC24->SetStartTime(Seconds(601.));
  appC24->SetStopTime(Seconds(701.));

//-----------------------------------------------------------------------------------------------------------///25
  uint32_t m_sinkPort25 = 8075;
  //receiver
  Address sinkAddress25(InetSocketAddress (VANET::m_interfaces.GetAddress(101), m_sinkPort25)); 
  Ptr<MyAPPS> appS25 = CreateObject<MyAPPS>();
  appS25->Setup(sinkAddress25, m_sinkPort25, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(101)->AddApplication(appS25);    
  appS25->SetStartTime(Seconds(750.));
  appS25->SetStopTime(Seconds(850.));
  //sender
  Ptr<Socket> ns3UdpSocket25 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(89),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC25 = CreateObject<MyAPPC>();
  appC25->Setup(ns3UdpSocket25, sinkAddress25, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(89)->AddApplication(appC25);
  appC25->SetStartTime(Seconds(751.));
  appC25->SetStopTime(Seconds(850.));

//-----------------------------------------------------------------------------------------------------------///26
  uint32_t m_sinkPort26 = 8076;
  //receiver
  Address sinkAddress26(InetSocketAddress (VANET::m_interfaces.GetAddress(77), m_sinkPort26)); 
  Ptr<MyAPPS> appS26 = CreateObject<MyAPPS>();
  appS26->Setup(sinkAddress26, m_sinkPort26, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(77)->AddApplication(appS26);    
  appS26->SetStartTime(Seconds(500.));
  appS26->SetStopTime(Seconds(600.));
  //sender
  Ptr<Socket> ns3UdpSocket26 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(62),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC26 = CreateObject<MyAPPC>();
  appC26->Setup(ns3UdpSocket26, sinkAddress26, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(62)->AddApplication(appC26);
  appC26->SetStartTime(Seconds(501.));
  appC26->SetStopTime(Seconds(600.));

//-----------------------------------------------------------------------------------------------------------///27
  uint32_t m_sinkPort27 = 8077;
  //receiver
  Address sinkAddress27(InetSocketAddress (VANET::m_interfaces.GetAddress(118), m_sinkPort27)); 
  Ptr<MyAPPS> appS27 = CreateObject<MyAPPS>();
  appS27->Setup(sinkAddress27, m_sinkPort27, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(118)->AddApplication(appS27);    
  appS27->SetStartTime(Seconds(700.));
  appS27->SetStopTime(Seconds(800.));
  //sender
  Ptr<Socket> ns3UdpSocket27 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(86),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC27 = CreateObject<MyAPPC>();
  appC27->Setup(ns3UdpSocket27, sinkAddress27, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(86)->AddApplication(appC27);
  appC27->SetStartTime(Seconds(701.));
  appC27->SetStopTime(Seconds(800.));

//-----------------------------------------------------------------------------------------------------------///28
  uint32_t m_sinkPort28 = 8078;
  //receiver
  Address sinkAddress28(InetSocketAddress (VANET::m_interfaces.GetAddress(149), m_sinkPort28)); 
  Ptr<MyAPPS> appS28 = CreateObject<MyAPPS>();
  appS28->Setup(sinkAddress28, m_sinkPort28, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(149)->AddApplication(appS28);    
  appS28->SetStartTime(Seconds(750.));
  appS28->SetStopTime(Seconds(850.));
  //sender
  Ptr<Socket> ns3UdpSocket28 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(137),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC28 = CreateObject<MyAPPC>();
  appC28->Setup(ns3UdpSocket28, sinkAddress28, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(137)->AddApplication(appC28);
  appC28->SetStartTime(Seconds(751.));
  appC28->SetStopTime(Seconds(850.));

//-----------------------------------------------------------------------------------------------------------///29
  uint32_t m_sinkPort29 = 8079;
  //receiver
  Address sinkAddress29(InetSocketAddress (VANET::m_interfaces.GetAddress(141), m_sinkPort29)); 
  Ptr<MyAPPS> appS29 = CreateObject<MyAPPS>();
  appS29->Setup(sinkAddress29, m_sinkPort29, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(141)->AddApplication(appS29);    
  appS29->SetStartTime(Seconds(900.));
  appS29->SetStopTime(Seconds(1100.));
  //sender
  Ptr<Socket> ns3UdpSocket29 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(129),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC29 = CreateObject<MyAPPC>();
  appC29->Setup(ns3UdpSocket29, sinkAddress29, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(129)->AddApplication(appC29);
  appC29->SetStartTime(Seconds(901.));
  appC29->SetStopTime(Seconds(1100.));

//-----------------------------------------------------------------------------------------------------------///30
  uint32_t m_sinkPort30 = 8080;
  //receiver
  Address sinkAddress30(InetSocketAddress (VANET::m_interfaces.GetAddress(144), m_sinkPort30)); 
  Ptr<MyAPPS> appS30 = CreateObject<MyAPPS>();
  appS30->Setup(sinkAddress30, m_sinkPort30, Pks_Len, DataRate("5Mbps"));
  VANET::m_vanetTxNodes.Get(144)->AddApplication(appS30);    
  appS30->SetStartTime(Seconds(900.));
  appS30->SetStopTime(Seconds(1000.));
  //sender
  Ptr<Socket> ns3UdpSocket30 = Socket::CreateSocket(VANET::m_vanetTxNodes.Get(134),UdpSocketFactory::GetTypeId());
  Ptr<MyAPPC> appC30 = CreateObject<MyAPPC>();
  appC30->Setup(ns3UdpSocket30, sinkAddress30, Pks_Len, DataRate("5Mbps")); 
  VANET::m_vanetTxNodes.Get(134)->AddApplication(appC30);
  appC30->SetStartTime(Seconds(901.));
  appC30->SetStopTime(Seconds(1000.));

//-----------------------------------------------------------------------------------------------------------/

  // Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  Simulator::Schedule(Seconds(50.0), &PrintDrop);

  Simulator::Stop (Seconds (1100.));
  Simulator::Run ();

  PrintDrop();

  // Print per flow statistics
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin (); iter != stats.end (); ++iter)
    {
	  Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first);

      if (( t.sourceAddress == Ipv4Address("10.1.1.2")  && t.destinationAddress == Ipv4Address("10.1.1.17"))
    	|| (t.sourceAddress == Ipv4Address("10.1.1.2")  && t.destinationAddress == Ipv4Address("10.1.1.27"))
        || (t.sourceAddress == Ipv4Address("10.1.1.31")  && t.destinationAddress == Ipv4Address("10.1.1.2")) 
        || (t.sourceAddress == Ipv4Address("10.1.1.3")  && t.destinationAddress == Ipv4Address("10.1.1.21"))
        || (t.sourceAddress == Ipv4Address("10.1.1.6")  && t.destinationAddress == Ipv4Address("10.1.1.25"))   
        || (t.sourceAddress == Ipv4Address("10.1.1.14")  && t.destinationAddress == Ipv4Address("10.1.1.21")) //6  
        || (t.sourceAddress == Ipv4Address("10.1.1.4")  && t.destinationAddress == Ipv4Address("10.1.1.32")) //7 
        || (t.sourceAddress == Ipv4Address("10.1.1.20")  && t.destinationAddress == Ipv4Address("10.1.1.45"))  
        || (t.sourceAddress == Ipv4Address("10.1.1.41")  && t.destinationAddress == Ipv4Address("10.1.1.55"))   
        || (t.sourceAddress == Ipv4Address("10.1.1.35")  && t.destinationAddress == Ipv4Address("10.1.1.62"))  //10

        || (t.sourceAddress == Ipv4Address("10.1.1.57")  && t.destinationAddress == Ipv4Address("10.1.1.71"))
    	|| (t.sourceAddress == Ipv4Address("10.1.1.69")  && t.destinationAddress == Ipv4Address("10.1.1.82"))
        || (t.sourceAddress == Ipv4Address("10.1.1.51")  && t.destinationAddress == Ipv4Address("10.1.1.76")) 
        || (t.sourceAddress == Ipv4Address("10.1.1.78")  && t.destinationAddress == Ipv4Address("10.1.1.108")) //14
        || (t.sourceAddress == Ipv4Address("10.1.1.93")  && t.destinationAddress == Ipv4Address("10.1.1.98"))   
        || (t.sourceAddress == Ipv4Address("10.1.1.103")  && t.destinationAddress == Ipv4Address("10.1.1.112")) //16 
        || (t.sourceAddress == Ipv4Address("10.1.1.116")  && t.destinationAddress == Ipv4Address("10.1.1.129"))   
        || (t.sourceAddress == Ipv4Address("10.1.1.124")  && t.destinationAddress == Ipv4Address("10.1.1.113"))  
        || (t.sourceAddress == Ipv4Address("10.1.1.107")  && t.destinationAddress == Ipv4Address("10.1.1.119"))   //4,13
        || (t.sourceAddress == Ipv4Address("10.1.1.136")  && t.destinationAddress == Ipv4Address("10.1.1.146")) 

        || (t.sourceAddress == Ipv4Address("10.1.1.13")  && t.destinationAddress == Ipv4Address("10.1.1.19"))
    	|| (t.sourceAddress == Ipv4Address("10.1.1.40")  && t.destinationAddress == Ipv4Address("10.1.1.44"))
        || (t.sourceAddress == Ipv4Address("10.1.1.59")  && t.destinationAddress == Ipv4Address("10.1.1.72")) 
        || (t.sourceAddress == Ipv4Address("10.1.1.94")  && t.destinationAddress == Ipv4Address("10.1.1.122")) //14
        || (t.sourceAddress == Ipv4Address("10.1.1.90")  && t.destinationAddress == Ipv4Address("10.1.1.102"))   
        || (t.sourceAddress == Ipv4Address("10.1.1.63")  && t.destinationAddress == Ipv4Address("10.1.1.78")) //16 
        || (t.sourceAddress == Ipv4Address("10.1.1.87")  && t.destinationAddress == Ipv4Address("10.1.1.119"))   
        || (t.sourceAddress == Ipv4Address("10.1.1.138")  && t.destinationAddress == Ipv4Address("10.1.1.150"))  
        || (t.sourceAddress == Ipv4Address("10.1.1.130")  && t.destinationAddress == Ipv4Address("10.1.1.142"))   
        || (t.sourceAddress == Ipv4Address("10.1.1.135")  && t.destinationAddress == Ipv4Address("10.1.1.145")) )

        {
    	  NS_LOG_UNCOND("Flow ID: " << iter->first << " Src Addr " << t.sourceAddress << " Dst Addr " << t.destinationAddress);
    	  NS_LOG_UNCOND("Tx Packets = " << iter->second.txPackets);
    	  NS_LOG_UNCOND("Rx Packets = " << iter->second.rxPackets);
    	  NS_LOG_UNCOND("Throughput: " << iter->second.rxBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds()) / 1000  << " Kbps");
        }
    }

  monitor->SerializeToXmlFile("lab-5.flowmon", true, true);

  Simulator::Destroy ();

}
//------------------------------------------------------------------

int main(int argc, char *argv[])
{
//******************************************************************
  std::cout << "Usage of  " << argv[0] << " :\n\n"
            "./waf --run '/scratch/1-4-bus-mobility --PrintHelp' --vis \n" 
            "Designed by Yohanna.WANG (finished in 2017-1-5 15:00 pm-) \n"
            "60 bus nodes with duration of 319.90s(HK MongKok District)\n";
//******************************************************************
  VANET vanet;
  vanet.CommandSetup(argc, argv);
  vanet.Simulate();
  SetupApplication();
  return 0;
}

