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
 * Finished in 2017-1-5 && Modified in 2017-2-20
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

  std::ifstream fsend("Vehicle.mp4", std::ifstream::binary);//*****************************************************************************
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
//--------------------------------------------------------------------------
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
    m_duration(1199.90),
    m_TxRange(300.0),
    m_nNodes(199),
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
  cmd.AddValue ("lossModel", "1=Friis;2=ItuR1411Los;3=TwoRayGround;4=LogDistance;5=Building", m_lossModel);
  cmd.AddValue ("fading", "0=None;1=Nakagami;(buildings=1 overrides)", m_fading);
  cmd.AddValue ("m_nNodes", "Number of nodes(110)", m_nNodes);
  cmd.AddValue ("m_duration", "Duration of Simulation(1199.90)", m_duration);
  cmd.AddValue ("mobility", "1=BUS,2=Vehicles,3=BUS+Vehicles", m_mobility);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("ascii", "Turn on ASCII trace function", ascii);
  cmd.AddValue ("pcap", "Turn on PCAP trace function", pcap);
  cmd.AddValue ("verbose", "0=quiet;1=verbose", m_verbose);
  cmd.AddValue ("protocol", "1=OLSR;2=AODV;3=DSDV;4=GPSR;5=DSP", m_protocol);
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
    //This is bus-only mobility trace
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
//-----------------Building Propagation loss-------------------//
  else if (m_lossModel == 5)
    {
      m_lossModelName = "ns3::HybridBuildingsPropagationLossModel";
    }
//--------------------------------------------------------------//
  else
    {
      NS_LOG_ERROR ("Invalid propagation loss model specified.Values must be [1-5],where                  1=Friis;2=ItuR1411Los;3=TwoRayGround;4=LogDistance;5=HybridBuilding");
    }
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

//--------------------------------------------------------------------------------------//
  // 802.11p 5.9 GHz
  double freq = 5.9e9;

  // Setup propagation models
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel","MaxRange", DoubleValue (m_TxRange));  //300.0
  if (m_lossModel == 3)
    {
      // two-ray requires antenna height (else defaults to Friss)
      wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq), "HeightAboveZ", DoubleValue (1.5));
    }
  if (m_lossModel == 5)
    {
      //Parameters required in the Hybrid building propagation loss model
      wifiChannel.AddPropagationLoss (m_lossModelName,
                                      "Frequency", DoubleValue (freq),
                                      "Environment", StringValue("Urban"),
                                      "CitySize", StringValue("Small"),
                                      "ShadowSigmaOutdoor", DoubleValue (7.0), 
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
      m_traceFile = "scrach/528-400-vehicle_trace.tcl";//109_bus_mobility.tcl
      m_logFile = "109bus_mobility.log";
      m_mobility = 1;
      m_nNodes = 199;  //60
      m_duration = 1199.9;  //319.90
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
//---------------------------------------Group-1----------------------------------------------//1
  uint32_t m_sinkPort = 8081;
  //receiver
  Address sinkAddress(InetSocketAddress(VANET::m_interfaces.GetAddress(16), m_sinkPort));
  PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort));
  ApplicationContainer sinkApps = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (16));
  sinkApps.Start (Seconds (25.));
  sinkApps.Stop (Seconds (200.));
 
  Ptr<Socket> ns3UdpSocket1 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (1), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app = CreateObject<MyAPPC> ();
  app->Setup (ns3UdpSocket1, sinkAddress, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (1)->AddApplication (app);
  app->SetStartTime (Seconds (26.));
  app->SetStopTime (Seconds (200.));

//---------------------------------------Group-2----------------------------------------------//2
  uint32_t m_sinkPort2 = 8082;
  //receiver
  Address sinkAddress2(InetSocketAddress(VANET::m_interfaces.GetAddress(26), m_sinkPort2));
  PacketSinkHelper packetSinkHelper2 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort2));
  ApplicationContainer sinkApps2 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (26));
  sinkApps2.Start (Seconds (50.));
  sinkApps2.Stop (Seconds (200.));
 
  //sender
  Ptr<Socket> ns3UdpSocket2 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (1), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app2 = CreateObject<MyAPPC> ();
  app2->Setup (ns3UdpSocket2, sinkAddress2, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (1)->AddApplication (app2);
  app2->SetStartTime (Seconds (51.));
  app2->SetStopTime (Seconds (200.));
//---------------------------------------Group-3----------------------------------------------//3
  uint32_t m_sinkPort3 = 8083;
  //receiver
  Address sinkAddress3(InetSocketAddress(VANET::m_interfaces.GetAddress(1), m_sinkPort3));
  PacketSinkHelper packetSinkHelper3 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort3));
  ApplicationContainer sinkApps3 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (1));
  sinkApps3.Start (Seconds (120.));
  sinkApps3.Stop (Seconds (200.));
 
  //sender
  Ptr<Socket> ns3UdpSocket3 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (30), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app3 = CreateObject<MyAPPC> ();
  app3->Setup (ns3UdpSocket3, sinkAddress3, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (30)->AddApplication (app3);
  app3->SetStartTime (Seconds (121.));
  app3->SetStopTime (Seconds (200.));
//--------------------------------------------------------------------------------------------//4
  uint32_t m_sinkPort4 = 8084;
  //receiver
  Address sinkAddress4(InetSocketAddress(VANET::m_interfaces.GetAddress(20), m_sinkPort4));
  PacketSinkHelper packetSinkHelper4 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort4));
  ApplicationContainer sinkApps4 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (20));
  sinkApps4.Start (Seconds (50.));
  sinkApps4.Stop (Seconds (250.));
 
  //sender
  Ptr<Socket> ns3UdpSocket4 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (2), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app4 = CreateObject<MyAPPC> ();
  app4->Setup (ns3UdpSocket4, sinkAddress4, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (2)->AddApplication (app4);
  app4->SetStartTime (Seconds (51.));
  app4->SetStopTime (Seconds (250.));
//--------------------------------------------------------------------------------------------//5

  uint32_t m_sinkPort5 = 8085;
  //receiver
  Address sinkAddress5(InetSocketAddress(VANET::m_interfaces.GetAddress(24), m_sinkPort5));
  PacketSinkHelper packetSinkHelper5 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort5));
  ApplicationContainer sinkApps5 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (24));
  sinkApps5.Start (Seconds (100.));
  sinkApps5.Stop (Seconds (250.));
 
  //sender
  Ptr<Socket> ns3UdpSocket5 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (5), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app5 = CreateObject<MyAPPC> ();
  app5->Setup (ns3UdpSocket5, sinkAddress5, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (5)->AddApplication (app5);
  app5->SetStartTime (Seconds (101.));
  app5->SetStopTime (Seconds (250.));

//--------------------------------------------------------------------------------------------//6
  uint32_t m_sinkPort6 = 8086;
  //receiver
  Address sinkAddress6(InetSocketAddress(VANET::m_interfaces.GetAddress(20), m_sinkPort6));
  PacketSinkHelper packetSinkHelper6 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort6));
  ApplicationContainer sinkApps6 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (20));
  sinkApps6.Start (Seconds (200.));
  sinkApps6.Stop (Seconds (350.));
 
  //sender
  Ptr<Socket> ns3UdpSocket6 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (13), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app6 = CreateObject<MyAPPC> ();
  app6->Setup (ns3UdpSocket6, sinkAddress6, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (13)->AddApplication (app6);
  app6->SetStartTime (Seconds (201.));
  app6->SetStopTime (Seconds (350.));
//--------------------------------------------------------------------------------------------//7
  uint32_t m_sinkPort7 = 8087;
  //receiver
  Address sinkAddress7(InetSocketAddress(VANET::m_interfaces.GetAddress(31), m_sinkPort7));
  PacketSinkHelper packetSinkHelper7 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort7));
  ApplicationContainer sinkApps7 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (31));
  sinkApps7.Start (Seconds (100.));
  sinkApps7.Stop (Seconds (250.));
 
  //sender
  Ptr<Socket> ns3UdpSocket7 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (3), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app7 = CreateObject<MyAPPC> ();
  app7->Setup (ns3UdpSocket7, sinkAddress7, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (3)->AddApplication (app7);
  app7->SetStartTime (Seconds (101.));
  app7->SetStopTime (Seconds (250.));
//--------------------------------------------------------------------------------------------//8
  uint32_t m_sinkPort8 = 8088;
  //receiver
  Address sinkAddress8(InetSocketAddress(VANET::m_interfaces.GetAddress(44), m_sinkPort8));
  PacketSinkHelper packetSinkHelper8 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort8));
  ApplicationContainer sinkApps8 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (44));
  sinkApps8.Start (Seconds (300.));
  sinkApps8.Stop (Seconds (400.));
 
  //sender
  Ptr<Socket> ns3UdpSocket8 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (19), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app8 = CreateObject<MyAPPC> ();
  app8->Setup (ns3UdpSocket8, sinkAddress8, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (19)->AddApplication (app8);
  app8->SetStartTime (Seconds (301.));
  app8->SetStopTime (Seconds (400.));
//--------------------------------------------------------------------------------------------//9

  uint32_t m_sinkPort9 = 8089;
  //receiver
  Address sinkAddress9(InetSocketAddress(VANET::m_interfaces.GetAddress(54), m_sinkPort9));
  PacketSinkHelper packetSinkHelper9 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort9));
  ApplicationContainer sinkApps9 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (54));
  sinkApps9.Start (Seconds (400.));
  sinkApps9.Stop (Seconds (500.));
 
  //sender
  Ptr<Socket> ns3UdpSocket9 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (40), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app9 = CreateObject<MyAPPC> ();
  app9->Setup (ns3UdpSocket9, sinkAddress9, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (40)->AddApplication (app9);
  app9->SetStartTime (Seconds (401.));
  app9->SetStopTime (Seconds (500.));

//--------------------------------------------------------------------------------------------//10
  uint32_t m_sinkPort10 = 8090;
  //receiver
  Address sinkAddress10(InetSocketAddress(VANET::m_interfaces.GetAddress(61), m_sinkPort10));
  PacketSinkHelper packetSinkHelper10 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort10));
  ApplicationContainer sinkApps10 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (61));
  sinkApps10.Start (Seconds (300.));
  sinkApps10.Stop (Seconds (400.));
 
  //sender
  Ptr<Socket> ns3UdpSocket10 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (34), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app10 = CreateObject<MyAPPC> ();
  app10->Setup (ns3UdpSocket10, sinkAddress10, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (34)->AddApplication (app10);
  app10->SetStartTime (Seconds (301.));
  app10->SetStopTime (Seconds (400.));
//--------------------------------------------------------------------------------------------//11
  uint32_t m_sinkPort11 = 8091;
  //receiver
  Address sinkAddress11(InetSocketAddress(VANET::m_interfaces.GetAddress(70), m_sinkPort11));
  PacketSinkHelper packetSinkHelper11 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort11));
  ApplicationContainer sinkApps11 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (70));
  sinkApps11.Start (Seconds (500.));
  sinkApps11.Stop (Seconds (600.));
 
  //sender
  Ptr<Socket> ns3UdpSocket11 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (56), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app11 = CreateObject<MyAPPC> ();
  app11->Setup (ns3UdpSocket11, sinkAddress11, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (56)->AddApplication (app11);
  app11->SetStartTime (Seconds (501.));
  app11->SetStopTime (Seconds (600.));
//--------------------------------------------------------------------------------------------//12
  uint32_t m_sinkPort12 = 8092;
  //receiver
  Address sinkAddress12(InetSocketAddress(VANET::m_interfaces.GetAddress(81), m_sinkPort12));
  PacketSinkHelper packetSinkHelper12 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort12));
  ApplicationContainer sinkApps12 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (81));
  sinkApps12.Start (Seconds (300.));
  sinkApps12.Stop (Seconds (500.));
 
  //sender
  Ptr<Socket> ns3UdpSocket12 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (68), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app12 = CreateObject<MyAPPC> ();
  app12->Setup (ns3UdpSocket12, sinkAddress12, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (68)->AddApplication (app12);
  app12->SetStartTime (Seconds (301.));
  app12->SetStopTime (Seconds (501.));
//--------------------------------------------------------------------------------------------//13
  uint32_t m_sinkPort13 = 8093;
  //receiver
  Address sinkAddress13(InetSocketAddress(VANET::m_interfaces.GetAddress(75), m_sinkPort13));
  PacketSinkHelper packetSinkHelper13 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort13));
  ApplicationContainer sinkApps13 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (75));
  sinkApps13.Start (Seconds (400.));
  sinkApps13.Stop (Seconds (500.));
 
  //sender
  Ptr<Socket> ns3UdpSocket13 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (50), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app13 = CreateObject<MyAPPC> ();
  app13->Setup (ns3UdpSocket13, sinkAddress13, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (50)->AddApplication (app13);
  app13->SetStartTime (Seconds (401.));
  app13->SetStopTime (Seconds (500.));
//--------------------------------------------------------------------------------------------//14
  uint32_t m_sinkPort14 = 8094;
  //receiver
  Address sinkAddress14(InetSocketAddress(VANET::m_interfaces.GetAddress(107), m_sinkPort14));
  PacketSinkHelper packetSinkHelper14 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort14));
  ApplicationContainer sinkApps14 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (107));
  sinkApps14.Start (Seconds (500.));
  sinkApps14.Stop (Seconds (700.));
 
  //sender
  Ptr<Socket> ns3UdpSocket14 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (77), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app14 = CreateObject<MyAPPC> ();
  app14->Setup (ns3UdpSocket14, sinkAddress14, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (77)->AddApplication (app14);
  app14->SetStartTime (Seconds (501.));
  app14->SetStopTime (Seconds (700.));
//--------------------------------------------------------------------------------------------//15
  uint32_t m_sinkPort15 = 8095;
  //receiver
  Address sinkAddress15(InetSocketAddress(VANET::m_interfaces.GetAddress(97), m_sinkPort15));
  PacketSinkHelper packetSinkHelper15 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort15));
  ApplicationContainer sinkApps15 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (97));
  sinkApps15.Start (Seconds (450.));
  sinkApps15.Stop (Seconds (550.));
 
  //sender
  Ptr<Socket> ns3UdpSocket15 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (92), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app15 = CreateObject<MyAPPC> ();
  app15->Setup (ns3UdpSocket15, sinkAddress15, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (92)->AddApplication (app15);
  app15->SetStartTime (Seconds (451.));
  app15->SetStopTime (Seconds (550.));
//--------------------------------------------------------------------------------------------//16
  uint32_t m_sinkPort16 = 8096;
  //receiver
  Address sinkAddress16(InetSocketAddress(VANET::m_interfaces.GetAddress(111), m_sinkPort16));
  PacketSinkHelper packetSinkHelper16 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort16));
  ApplicationContainer sinkApps16 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (111));
  sinkApps16.Start (Seconds (550.));
  sinkApps16.Stop (Seconds (700.));
 
  //sender
  Ptr<Socket> ns3UdpSocket16 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (102), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app16 = CreateObject<MyAPPC> ();
  app16->Setup (ns3UdpSocket16, sinkAddress16, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (102)->AddApplication (app16);
  app16->SetStartTime (Seconds (551.));
  app16->SetStopTime (Seconds (700.));
//--------------------------------------------------------------------------------------------//17
  uint32_t m_sinkPort17 = 8097;
  //receiver
  Address sinkAddress17(InetSocketAddress(VANET::m_interfaces.GetAddress(128), m_sinkPort17));
  PacketSinkHelper packetSinkHelper17 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort17));
  ApplicationContainer sinkApps17 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (128));
  sinkApps17.Start (Seconds (700.));
  sinkApps17.Stop (Seconds (900.));
 
  //sender
  Ptr<Socket> ns3UdpSocket17 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (115), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app17 = CreateObject<MyAPPC> ();
  app17->Setup (ns3UdpSocket17, sinkAddress17, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (115)->AddApplication (app17);
  app17->SetStartTime (Seconds (701.));
  app17->SetStopTime (Seconds (900.));
//--------------------------------------------------------------------------------------------//18
  uint32_t m_sinkPort18 = 8098;
  //receiver
  Address sinkAddress18(InetSocketAddress(VANET::m_interfaces.GetAddress(112), m_sinkPort18));
  PacketSinkHelper packetSinkHelper18 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort18));
  ApplicationContainer sinkApps18 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (112));
  sinkApps18.Start (Seconds (650.));
  sinkApps18.Stop (Seconds (800.));
 
  //sender
  Ptr<Socket> ns3UdpSocket18 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (123), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app18 = CreateObject<MyAPPC> ();
  app18->Setup (ns3UdpSocket18, sinkAddress18, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (123)->AddApplication (app18);
  app18->SetStartTime (Seconds (651.));
  app18->SetStopTime (Seconds (800.));
//--------------------------------------------------------------------------------------------//19
  uint32_t m_sinkPort19 = 8099;
  //receiver
  Address sinkAddress19(InetSocketAddress(VANET::m_interfaces.GetAddress(118), m_sinkPort19));
  PacketSinkHelper packetSinkHelper19 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort19));
  ApplicationContainer sinkApps19 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (118));
  sinkApps19.Start (Seconds (800.));
  sinkApps19.Stop (Seconds (900.));
 
  //sender
  Ptr<Socket> ns3UdpSocket19 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (106), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app19 = CreateObject<MyAPPC> ();
  app19->Setup (ns3UdpSocket19, sinkAddress19, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (106)->AddApplication (app19);
  app19->SetStartTime (Seconds (801.));
  app19->SetStopTime (Seconds (900.));
//--------------------------------------------------------------------------------------------//20
  uint32_t m_sinkPort20 = 8100;
  //receiver
  Address sinkAddress20(InetSocketAddress(VANET::m_interfaces.GetAddress(145), m_sinkPort20));
  PacketSinkHelper packetSinkHelper20 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort20));
  ApplicationContainer sinkApps20 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (145));
  sinkApps20.Start (Seconds (700.));
  sinkApps20.Stop (Seconds (800.));
 
  //sender
  Ptr<Socket> ns3UdpSocket20 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (135), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app20 = CreateObject<MyAPPC> ();
  app20->Setup (ns3UdpSocket20, sinkAddress20, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (135)->AddApplication (app20);
  app20->SetStartTime (Seconds (701.));
  app20->SetStopTime (Seconds (800.));
//--------------------------------------------------------------------------------------------//21
  uint32_t m_sinkPort21 = 8101;
  //receiver
  Address sinkAddress21(InetSocketAddress(VANET::m_interfaces.GetAddress(18), m_sinkPort21));
  PacketSinkHelper packetSinkHelper21 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort21));
  ApplicationContainer sinkApps21 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (18));
  sinkApps21.Start (Seconds (250.));
  sinkApps21.Stop (Seconds (350.));
 
  //sender
  Ptr<Socket> ns3UdpSocket21 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (12), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app21 = CreateObject<MyAPPC> ();
  app21->Setup (ns3UdpSocket21, sinkAddress21, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (12)->AddApplication (app21);
  app21->SetStartTime (Seconds (251.));
  app21->SetStopTime (Seconds (350.));

//--------------------------------------------------------------------------------------------//22
  uint32_t m_sinkPort22 = 8102;
  //receiver
  Address sinkAddress22(InetSocketAddress(VANET::m_interfaces.GetAddress(43), m_sinkPort22));
  PacketSinkHelper packetSinkHelper22 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort22));
  ApplicationContainer sinkApps22 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (43));
  sinkApps22.Start (Seconds (400.));
  sinkApps22.Stop (Seconds (500.));
 
  //sender
  Ptr<Socket> ns3UdpSocket22 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (39), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app22 = CreateObject<MyAPPC> ();
  app22->Setup (ns3UdpSocket22, sinkAddress22, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (39)->AddApplication (app22);
  app22->SetStartTime (Seconds (401.));
  app22->SetStopTime (Seconds (500.));
//--------------------------------------------------------------------------------------------//23
  uint32_t m_sinkPort23 = 8103;
  //receiver
  Address sinkAddress23(InetSocketAddress(VANET::m_interfaces.GetAddress(71), m_sinkPort23));
  PacketSinkHelper packetSinkHelper23 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort23));
  ApplicationContainer sinkApps23 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (71));
  sinkApps23.Start (Seconds (450.));
  sinkApps23.Stop (Seconds (550.));
 
  //sender
  Ptr<Socket> ns3UdpSocket23 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (58), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app23 = CreateObject<MyAPPC> ();
  app23->Setup (ns3UdpSocket23, sinkAddress23, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (58)->AddApplication (app23);
  app23->SetStartTime (Seconds (451.));
  app23->SetStopTime (Seconds (550.));

//--------------------------------------------------------------------------------------------//24
  uint32_t m_sinkPort24 = 8104;
  //receiver
  Address sinkAddress24(InetSocketAddress(VANET::m_interfaces.GetAddress(121), m_sinkPort24));
  PacketSinkHelper packetSinkHelper24 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort24));
  ApplicationContainer sinkApps24 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (121));
  sinkApps24.Start (Seconds (600.));
  sinkApps24.Stop (Seconds (700.));
 
  //sender
  Ptr<Socket> ns3UdpSocket24 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (93), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app24 = CreateObject<MyAPPC> ();
  app24->Setup (ns3UdpSocket24, sinkAddress24, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (93)->AddApplication (app24);
  app24->SetStartTime (Seconds (601.));
  app24->SetStopTime (Seconds (700.));
//--------------------------------------------------------------------------------------------//25
  uint32_t m_sinkPort25 = 8105;
  //receiver
  Address sinkAddress25(InetSocketAddress(VANET::m_interfaces.GetAddress(101), m_sinkPort25));
  PacketSinkHelper packetSinkHelper25 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort25));
  ApplicationContainer sinkApps25 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (101));
  sinkApps25.Start (Seconds (750.));
  sinkApps25.Stop (Seconds (850.));
 
  //sender
  Ptr<Socket> ns3UdpSocket25 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (89), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app25 = CreateObject<MyAPPC> ();
  app25->Setup (ns3UdpSocket25, sinkAddress25, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (89)->AddApplication (app25);
  app25->SetStartTime (Seconds (751.));
  app25->SetStopTime (Seconds (850.));

//--------------------------------------------------------------------------------------------//26
  uint32_t m_sinkPort26 = 8106;
  //receiver
  Address sinkAddress26(InetSocketAddress(VANET::m_interfaces.GetAddress(77), m_sinkPort26));
  PacketSinkHelper packetSinkHelper26 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort26));
  ApplicationContainer sinkApps26 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (77));
  sinkApps26.Start (Seconds (500.));
  sinkApps26.Stop (Seconds (600.));
 
  //sender
  Ptr<Socket> ns3UdpSocket26 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (62), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app26 = CreateObject<MyAPPC> ();
  app26->Setup (ns3UdpSocket26, sinkAddress26, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (62)->AddApplication (app26);
  app26->SetStartTime (Seconds (501.));
  app26->SetStopTime (Seconds (600.));

//--------------------------------------------------------------------------------------------//27
  uint32_t m_sinkPort27 = 8107;
  //receiver
  Address sinkAddress27(InetSocketAddress(VANET::m_interfaces.GetAddress(118), m_sinkPort27));
  PacketSinkHelper packetSinkHelper27 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort27));
  ApplicationContainer sinkApps27 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (118));
  sinkApps27.Start (Seconds (700.));
  sinkApps27.Stop (Seconds (800.));
 
  //sender
  Ptr<Socket> ns3UdpSocket27 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (86), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app27 = CreateObject<MyAPPC> ();
  app27->Setup (ns3UdpSocket27, sinkAddress27, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (86)->AddApplication (app27);
  app27->SetStartTime (Seconds (701.));
  app27->SetStopTime (Seconds (800.));

//--------------------------------------------------------------------------------------------//28
  uint32_t m_sinkPort28 = 8108;
  //receiver
  Address sinkAddress28(InetSocketAddress(VANET::m_interfaces.GetAddress(149), m_sinkPort28));
  PacketSinkHelper packetSinkHelper28 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort28));
  ApplicationContainer sinkApps28 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (149));
  sinkApps28.Start (Seconds (750.));
  sinkApps28.Stop (Seconds (850.));
 
  //sender
  Ptr<Socket> ns3UdpSocket28 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (137), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app28 = CreateObject<MyAPPC> ();
  app28->Setup (ns3UdpSocket28, sinkAddress28, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (137)->AddApplication (app28);
  app28->SetStartTime (Seconds (751.));
  app28->SetStopTime (Seconds (850.));

//--------------------------------------------------------------------------------------------//29
  uint32_t m_sinkPort29 = 8109;
  //receiver
  Address sinkAddress29(InetSocketAddress(VANET::m_interfaces.GetAddress(141), m_sinkPort29));
  PacketSinkHelper packetSinkHelper29 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort29));
  ApplicationContainer sinkApps29 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (141));
  sinkApps29.Start (Seconds (900.));
  sinkApps29.Stop (Seconds (1100.));
 
  //sender
  Ptr<Socket> ns3UdpSocket29 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (129), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app29 = CreateObject<MyAPPC> ();
  app29->Setup (ns3UdpSocket29, sinkAddress29, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (129)->AddApplication (app29);
  app29->SetStartTime (Seconds (901.));
  app29->SetStopTime (Seconds (1100.));

//--------------------------------------------------------------------------------------------//30
  uint32_t m_sinkPort30 = 8110;
  //receiver
  Address sinkAddress30(InetSocketAddress(VANET::m_interfaces.GetAddress(144), m_sinkPort30));
  PacketSinkHelper packetSinkHelper30 ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), m_sinkPort30));
  ApplicationContainer sinkApps30 = packetSinkHelper.Install (VANET::m_vanetTxNodes.Get (144));
  sinkApps30.Start (Seconds (900.));
  sinkApps30.Stop (Seconds (1000.));
 
  //sender
  Ptr<Socket> ns3UdpSocket30 = Socket::CreateSocket (VANET::m_vanetTxNodes.Get (134), UdpSocketFactory::GetTypeId ());
  Ptr<MyAPPC> app30 = CreateObject<MyAPPC> ();
  app30->Setup (ns3UdpSocket30, sinkAddress30, Pks_Len, DataRate ("5Mbps"));
  VANET::m_vanetTxNodes.Get (134)->AddApplication (app30);
  app30->SetStartTime (Seconds (901.));
  app30->SetStopTime (Seconds (1000.));

///----------------
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
    	  NS_LOG_UNCOND("Throughput: " << iter->second.rxBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds()) / 1024  << " Kbps");
        }
    }
  monitor->SerializeToXmlFile("lab.flowmon", true, true);

  Simulator::Destroy ();
}

//------------------------------------------------------------------

int main (int argc, char *argv[])
{
//******************************************************************
  std::cout << "Usage of  " << argv[0] << " :\n\n"
            "./waf --run '/scratch/1-4-bus-mobility --PrintHelp' --vis \n" 
            "This is a test of TCP(Udp)+Data dissemination+VANET+SUMO Trace \n"
            "Designed by Yohanna.WANG (finished in 2017-1-6 11:23 am-) \n"
            "60 bus nodes with duration of 319.90s(HK MongKok District)\n";
//******************************************************************
  VANET vanet;
  vanet.CommandSetup(argc, argv);
  vanet.Simulate();
  SetupApplication();  

  return 0;
}
