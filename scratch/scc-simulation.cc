/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2021 NYU WIRELESS,
 * New York University Tandon School of Engineering
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
 */

/**
* This program simulates a one UE and one BS scenario for the SCC Project.
* This program adopts the 3GPP channel model (3GPP TR 38.901) implemented in the
* last version of this module.
* The simulation involves a UE which is randomly walking in the surroundings
* of a BS (gNB). The wireless channel between the UE and the BS is at 28 GHz
* with 100 MHz of bandwidth.
* Also, the UE generates uplink-CBR traffic towards a Remote/Edge Server.
* The default propagation environment for this scenario is UMi.
*/

#include <fstream>
#include <numeric>
#include "ns3/core-module.h"
#include "ns3/trace-helper.h"
#include "ns3/mmwave-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/buildings-module.h"
#include "ns3/applications-module.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/mobility-model.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"

NS_LOG_COMPONENT_DEFINE ("SccSimulationScenario");

using namespace ns3;
using namespace mmwave;

uint32_t timeRes = 100;
Ptr<OutputStreamWrapper> stream1, stream2, stream3, stream4;

static void
ComputeStatistics (ApplicationContainer sinkApps, std::vector<uint32_t> lastRxBytes, std::vector<Ptr<MobilityModel>> mobilityModels)
{
  uint32_t totRxBytes, rxBytes;
  Ptr<PacketSink> sink;
  Vector pos;
  double now = Simulator::Now ().GetSeconds ();
  std::cout << now << "\t";
  *stream1->GetStream () << now;
  for (uint32_t i = 0; i < sinkApps.GetN (); ++i)
  {
    sink = StaticCast<PacketSink> (sinkApps.Get (i));
    totRxBytes = sink->GetTotalRx ();
    rxBytes = totRxBytes - lastRxBytes.at (i);
    lastRxBytes.at (i) = totRxBytes;
    double thr = rxBytes * 8.0 / (timeRes * 1e-3) / 1e6;  // Throughput in Mbps
    pos = mobilityModels.at (i)->GetPosition ();
    std::cout << thr << "\t";
    *stream1->GetStream () << "\t" << thr << "\t" << pos.x << "\t" << pos.y;
  }  
  std::cout << "Mbps" << std::endl;
  *stream1->GetStream () << std::endl;

  Simulator::Schedule (MilliSeconds (timeRes), &ComputeStatistics, sinkApps, lastRxBytes, mobilityModels);
}

static void
Rx (uint32_t appId, Ptr<const Packet> pkt, const Address &rxAddr, const Address &txAddr, const SeqTsSizeHeader& hdr)
{
  NS_LOG_DEBUG ("Rx packet with size: " << pkt->GetSize () << "; appId: " << appId); 
  Time now = Simulator::Now ();
  Time txTime = hdr.GetTs ();
  double pktDelay = now.GetSeconds () - txTime.GetSeconds ();
  NS_LOG_DEBUG ("Delay for packet with seq=" << hdr.GetSeq () << " is: " << pktDelay*1e3 << " ms");
  *stream2->GetStream () << appId << "\t" << Simulator::Now ().GetSeconds () << "\t" << hdr.GetSeq () << "\t" << pkt->GetSize () << "\t" << pktDelay << std::endl;
}

static void
Tx (uint32_t appId, Ptr<const Packet> pkt, const Address &rxAddr, const Address &txAddr, const SeqTsSizeHeader& hdr)
{
  NS_LOG_DEBUG ("Tx packet with size: " << pkt->GetSize () << "; appId: " << appId);
  *stream4->GetStream () << appId << "\t" << Simulator::Now ().GetSeconds () << "\t" << hdr.GetSeq () << "\t" << pkt->GetSize () << std::endl;
}

static void
Sinr (uint32_t ueId, uint64_t imsi, SpectrumValue& oldSinr, SpectrumValue& newSinr)
{
  double sinr = Sum (newSinr) / (newSinr.GetSpectrumModel ()->GetNumBands ());
  NS_LOG_DEBUG (Simulator::Now ().GetSeconds () << "\t" << 10*log10 (sinr) << " dB");
  *stream3->GetStream () << ueId << "\t" << Simulator::Now ().GetSeconds () << "\t" << sinr << std::endl;
}

int
main (int argc, char *argv[])
{
  bool harqEnabled = true;
  bool rlcAmEnabled = true;
  double bandwidth = 100e6; // bandwidth of the simulation [MHz]
  uint32_t appPacketSize = 1440; // application layer packet size [bytes]
  uint16_t enbAntennaNum = 64; // number of antenna elements at the BS
  uint16_t ueAntennaNum = 16; // number of antenna elements at the UE
  double frequency = 28e9; // operating frequency [Hz]
  double txPow = 30.0; // tx power [dBm]
  double noiseFigure = 9.0; // noise figure [dB]
  uint32_t simTime = 10; // simulation time [s]
  uint32_t updatePeriod = 100; // channel/channel condition update period [ms]
  uint16_t nonSelfBlocking = 4; // number of self-blocking components for the blockage model
  uint32_t remoteHostDelay = 10; // delay from PGW to remote host [ms]
  uint32_t uesPerBs = 2; // number of intended UEs associated to the BS
  uint32_t numberBs = 1; // number of gNBs in the simulation  
  uint32_t changeDirectionTime = 30; // trigger direction change for the random walk
  std::string outputFolder = ""; // path to the main output folder for the results
  std::string scenario = "UMi-StreetCanyon"; // 3GPP propagation scenario (Urban-Micro)
  bool isBlockage = false; // enable blockage modeling
  bool enableLog = false;


  CommandLine cmd;
  cmd.AddValue ("rlcAmEnabled", "Enable RLC AM mode at RLC layer", rlcAmEnabled);
  cmd.AddValue ("harqEnabled", "Enable HARQ at the MAC layer", harqEnabled);
  cmd.AddValue ("updatePeriod", "Channel/channel condition update periodicity [ms]", updatePeriod);
  cmd.AddValue ("blockage", "Enable blockage model A of the 3GPP channel model", isBlockage);
  cmd.AddValue ("nonSelfBlocking", "Number of non self-blocking components", nonSelfBlocking);
  cmd.AddValue ("uesPerBs", "Number of UE connected to each BS", uesPerBs);
  cmd.AddValue ("numBs", "Number of gNBs in the simulation", numberBs);
  cmd.AddValue ("simTime", "Simulation time [s]", simTime);
  cmd.Parse (argc, argv);

  if (enableLog)
  {
    LogComponentEnable ("RandomWalk2dOutdoor", LOG_LEVEL_ALL);
  }

  // setting power and noise figure
  Config::SetDefault ("ns3::MmWaveEnbPhy::TxPower", DoubleValue (txPow));
  Config::SetDefault ("ns3::MmWaveEnbPhy::NoiseFigure", DoubleValue (noiseFigure));
  Config::SetDefault ("ns3::MmWaveUePhy::TxPower", DoubleValue (txPow));
  Config::SetDefault ("ns3::MmWaveUePhy::NoiseFigure", DoubleValue (noiseFigure));

  // setting the 3GPP channel model
  Config::SetDefault ("ns3::ThreeGppChannelModel::Blockage", BooleanValue (isBlockage));
  Config::SetDefault ("ns3::ThreeGppChannelModel::NumNonselfBlocking", IntegerValue (nonSelfBlocking));
  Config::SetDefault ("ns3::ThreeGppChannelModel::BlockerSpeed", DoubleValue (1.5));
  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (updatePeriod)));
  Config::SetDefault ("ns3::ThreeGppChannelModel::PortraitMode", BooleanValue (false)); // blockage model with UT in landscape mode
  Config::SetDefault ("ns3::ThreeGppChannelConditionModel::UpdatePeriod", TimeValue (MilliSeconds (updatePeriod)));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue (bandwidth)); // fixed 100 MHz bandwidth
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (frequency));
  
  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue (MicroSeconds (100.0)));
  Config::SetDefault ("ns3::LteRlcUmLowLat::ReportBufferStatusTimer", TimeValue (MicroSeconds (100.0)));
  Config::SetDefault ("ns3::LteRlcUm::ReportBufferStatusTimer", TimeValue (MicroSeconds (100.0)));
  Config::SetDefault ("ns3::LteRlcUmLowLat::ReorderingTimeExpires", TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcUm::ReorderingTimer", TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcAm::ReorderingTimer", TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (10 * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcUmLowLat::MaxTxBufferSize", UintegerValue (10 * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue (10 * 1024 * 1024));

  Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::BeamformingModel", StringValue ("ns3::MmWaveSvdBeamforming"));

  std::cout << "rlcAmEnabled: " << rlcAmEnabled << std::endl;
  std::cout << "harqEnabled: " << harqEnabled << std::endl;
  std::cout << "updatePeriod: " << updatePeriod << std::endl;
  std::cout << "blockage: " << isBlockage << std::endl;
  std::cout << "nonSelfBlocking: " << nonSelfBlocking << std::endl;
  std::cout << "uesPerBs: " << uesPerBs << std::endl;
  std::cout << "numBs: " << numberBs << std::endl;
  std::cout << "simTime: " << simTime << std::endl;
  std::cout << "Seed: " << ns3::RngSeedManager::GetSeed () << std::endl;
  std::cout << "Run: " << ns3::RngSeedManager::GetRun () << std::endl;

  Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();

  // create and configure the factories for the channel condition and propagation loss models
  ObjectFactory propagationLossModelFactory;
  ObjectFactory channelConditionModelFactory;
  if (scenario == "RMa")
  {
    mmwaveHelper->SetPathlossModelType ("ns3::ThreeGppRmaPropagationLossModel");
    mmwaveHelper->SetChannelConditionModelType ("ns3::ThreeGppRmaChannelConditionModel");
  }
  else if (scenario == "UMa")
  {
    mmwaveHelper->SetPathlossModelType ("ns3::ThreeGppUmaPropagationLossModel");
    mmwaveHelper->SetChannelConditionModelType ("ns3::ThreeGppUmaChannelConditionModel");
  }
  else if (scenario == "UMi-StreetCanyon")
  {
    mmwaveHelper->SetPathlossModelType ("ns3::ThreeGppUmiStreetCanyonPropagationLossModel");
    mmwaveHelper->SetChannelConditionModelType ("ns3::ThreeGppUmiStreetCanyonChannelConditionModel");
  }
  else
  {
    NS_FATAL_ERROR ("Unknown scenario");
  }

  // setting antenna parameters
  mmwaveHelper->SetMmWaveEnbNetDeviceAttribute ("AntennaNum", UintegerValue (enbAntennaNum));
  mmwaveHelper->SetMmWaveUeNetDeviceAttribute ("AntennaNum", UintegerValue (ueAntennaNum));

  // setting scheduler
  mmwaveHelper->SetSchedulerType ("ns3::MmWaveFlexTtiMacScheduler");

  Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);
  mmwaveHelper->SetHarqEnabled (harqEnabled);
  mmwaveHelper->Initialize ();

  // get SGW/PGW and create the remote host
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // connect remote host and PGW
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (remoteHostDelay)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // create nodes
  NodeContainer enbNodes;
  NodeContainer ueNodes;
  enbNodes.Create (numberBs);
  ueNodes.Create (numberBs * uesPerBs);

  // assign mobility models to BSs
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  enbPositionAlloc->Add (Vector (-100.0, 0.0, 10.0)); // position of the intended BS
  enbPositionAlloc->Add (Vector (100.0, 0.0, 10.0)); // position of the interfering BS
  MobilityHelper mobilityHelper;
  mobilityHelper.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityHelper.SetPositionAllocator (enbPositionAlloc);
  for (uint32_t i = 0; i < enbNodes.GetN (); ++i)
  {
    mobilityHelper.Install (enbNodes.Get (i)); // install the mobility model in each BS
  }

  Ptr<UniformRandomVariable> uniform = CreateObject<UniformRandomVariable> ();
  double x,y;
  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
  {
    Ptr<MobilityModel> ueMobility = CreateObject<RandomWalk2dOutdoorMobilityModel> (); // 2D random walk mobility model for each UE
    ueMobility->SetAttribute ("Mode", EnumValue (RandomWalk2dOutdoorMobilityModel::MODE_TIME)); // updating mode for UE direction and speed
    ueMobility->SetAttribute ("Time", TimeValue (Seconds (changeDirectionTime)));
    //ueMobility->SetAttribute ("Distance", DoubleValue (2.0)); // update UE direction and speed every VALUE meters walked
    
    if (i < ueNodes.GetN ()/numberBs)
    {
      // intended UEs initial position
      x = 200.0 * uniform->GetValue () - 100.0;
      y = 200.0 * uniform->GetValue () - 100.0; 
      ueMobility->SetAttribute ("Bounds", RectangleValue (Rectangle (-100.0, 100.0, -100.0, 100.0))); // intended UEs random walk bounds     
    }
    else
    {
      // interfering UEs initial position
      x = 200.0 * uniform->GetValue () + 100.0;
      y = 200.0 * uniform->GetValue () - 100.0;
      ueMobility->SetAttribute ("Bounds", RectangleValue (Rectangle (100.0, 300.0, -100.0, 100.0)));
    }

    ueMobility->SetPosition (Vector (x, y, 1.6));
    ueNodes.Get (i)->AggregateObject (ueMobility);
  }
  
  // Create the tx and rx devices
  NetDeviceContainer enbMmWaveDevs = mmwaveHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueMmWaveDevs = mmwaveHelper->InstallUeDevice (ueNodes);

  Ptr<MmWaveEnbNetDevice> enbNetDevice;
  for (uint32_t i = 0; i < enbMmWaveDevs.GetN (); ++i)
  {
    enbNetDevice = StaticCast<MmWaveEnbNetDevice> (enbMmWaveDevs.Get (i));
    Ptr<ThreeGppAntennaArrayModel> antenna = enbNetDevice->GetPhy ()->GetDlSpectrumPhy ()->GetBeamformingModel ()->GetAntenna ();
    //antenna->SetAttribute ("DowntiltAngle", DoubleValue (DegreesToRadians (30.0)));
    //antenna->SetAttribute ("BearingAngle" , DoubleValue (DegreesToRadians (90.0)));
    antenna->SetAttribute ("IsotropicElements", BooleanValue (true));
  }

  Ptr<MmWaveUeNetDevice> ueNetDevice;
  for (uint32_t i = 0; i < ueMmWaveDevs.GetN (); ++i)
  {
    ueNetDevice = StaticCast<MmWaveUeNetDevice> (ueMmWaveDevs.Get (i));
    Ptr<ThreeGppAntennaArrayModel> antenna = ueNetDevice->GetPhy ()->GetDlSpectrumPhy ()->GetBeamformingModel ()->GetAntenna ();
    //antenna->SetAttribute ("BearingAngle" , DoubleValue (DegreesToRadians (-90.0)));
    antenna->SetAttribute ("IsotropicElements", BooleanValue (true));
  }

  // install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueMmWaveDevs)); // assign IP address to UEs
  Ptr<Node> ueNode = ueNodes.Get (0); 
  // set the default gateway for the UES
  Ptr<Ipv4StaticRouting> ueStaticRouting;
  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
  {
    ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNodes.Get (i)->GetObject<Ipv4> ());
    ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
  }
  
  // attach UEs to the Mmwave BSs
  mmwaveHelper->AttachToClosestEnb (ueMmWaveDevs, enbMmWaveDevs);

  uint16_t ulPort = 2000;
  std::vector<double> appStartTime (ueNodes.GetN ());
  ApplicationContainer clientInterfApps, clientIntendedApps;
  ApplicationContainer serverInterfApps, serverIntendedApps;
  // for each UE create a CBR application @100 Mbps (uplink traffic from the UE to the remote host) 
  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
  {
    ++ulPort;
    // create the sink 
    PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
    ulPacketSinkHelper.SetAttribute ("EnableSeqTsSizeHeader", BooleanValue (true)); // enable SeqTs header to measure end-to-end delay
    // create the CBR application
    OnOffHelper ulOnOffHelper ("ns3::UdpSocketFactory", InetSocketAddress (remoteHostAddr, ulPort));
    ulOnOffHelper.SetAttribute ("EnableSeqTsSizeHeader", BooleanValue (true)); // enable SeqTs header to measure end-to-end delay
    ulOnOffHelper.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=100000.0]"));
    ulOnOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
    ulOnOffHelper.SetAttribute ("DataRate", DataRateValue (DataRate ("100Mbps")));
    ulOnOffHelper.SetAttribute ("PacketSize", UintegerValue (appPacketSize));
    // randomize the starting time 
    appStartTime.at (i) = uniform->GetValue (0.1, 0.4);
    ulOnOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (appStartTime.at (i))));
    
    if (i < ueNodes.GetN ()/numberBs)
    {
      serverIntendedApps.Add (ulPacketSinkHelper.Install (remoteHost));
      clientIntendedApps.Add (ulOnOffHelper.Install (ueNodes.Get (i)));
    }
    else
    {
      serverInterfApps.Add (ulPacketSinkHelper.Install (remoteHost));
      clientInterfApps.Add (ulOnOffHelper.Install (ueNodes.Get (i)));
    }
  }

  // (OPTIONAL) enable output traces
  // mmwaveHelper->EnableTraces ();

  AsciiTraceHelper asciiTraceHelper;
  stream1 = asciiTraceHelper.CreateFileStream (outputFolder+"thr-mobility.csv");
  stream2 = asciiTraceHelper.CreateFileStream (outputFolder+"rx-packet-trace.csv");
  stream3 = asciiTraceHelper.CreateFileStream (outputFolder+"sinr-trace.csv");
  stream4 = asciiTraceHelper.CreateFileStream (outputFolder+"tx-packet-trace.csv");

  // SINR traces of the intended UEs
  uint32_t i = 0;
  Ptr<MmWaveUePhy> uePhy;
  while (i < ueMmWaveDevs.GetN ()/numberBs)
  {
    uePhy = StaticCast <MmWaveUeNetDevice> (ueMmWaveDevs.Get (i))->GetPhy ();
    uePhy->TraceConnectWithoutContext ("ReportCurrentCellRsrpSinr", MakeBoundCallback (&Sinr, i));
    i++;
  }

  // App layer traces: collect statistics of intended UEs
  for (uint32_t i = 0; i < serverIntendedApps.GetN (); ++i)
  {
    serverIntendedApps.Get (i)->TraceConnectWithoutContext ("RxWithSeqTsSize", MakeBoundCallback (&Rx, i));
  }

  for (uint32_t i = 0; i < clientIntendedApps.GetN (); ++i)
  {
    clientIntendedApps.Get (i)->TraceConnectWithoutContext ("TxWithSeqTsSize", MakeBoundCallback (&Tx, i));
  }

  // collect mobility of the UEs
  std::vector<Ptr<MobilityModel>> mobilityModels (ueNodes.GetN ()/numberBs);
  for (uint32_t i = 0; i < ueNodes.GetN ()/numberBs; ++i)
  {
    mobilityModels.at (i) = ueNodes.Get (i)->GetObject<MobilityModel> ();
    Vector initPos = mobilityModels.at (i)->GetPosition ();
    *stream1->GetStream () << 255.0 << "\t" << initPos.x << "\t" << initPos.y << std::endl;
  }

  // print applications start time
  for (uint32_t i = 0; i < appStartTime.size (); ++i)
  {
    *stream2->GetStream () << "\t" << appStartTime.at(i);
  }
  *stream2->GetStream () << std::endl;
  
  double maxAppStartTime = *std::max_element (appStartTime.begin (), appStartTime.end ());
  std::vector<uint32_t> lastRxBytes (serverIntendedApps.GetN (), 0);
  Simulator::Schedule (Seconds (maxAppStartTime), &ComputeStatistics, serverIntendedApps, lastRxBytes, mobilityModels);

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
