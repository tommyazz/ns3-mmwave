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

Ptr<PacketSink> sinkApp; // Sink application at the remote host
uint32_t timeRes = 10; // Time resolution for throughput calculation [ms]
uint32_t lastRxBytes = 0;
Ptr<OutputStreamWrapper> stream1, stream2, stream3;

static void
ComputeE2eThroughput ()
{
  uint32_t totRxBytes = sinkApp->GetTotalRx ();   // Total rx bytes
  uint32_t rxBytes = totRxBytes - lastRxBytes;
  lastRxBytes = totRxBytes;
  double thr = rxBytes * 8.0 / (timeRes * 1e-3) / 1e6;  // Throughput in Mbps
  NS_LOG_DEBUG (Simulator::Now ().GetSeconds () << "\t" << thr << " Mbps");
  *stream1->GetStream () << Simulator::Now ().GetSeconds () << "\t" << thr << std::endl;

  Simulator::Schedule (MilliSeconds (timeRes), &ComputeE2eThroughput);
}

static void
Rx (Ptr<const Packet> pkt, const Address &rxAddr, const Address &txAddr, const SeqTsSizeHeader& hdr)
{
  NS_LOG_DEBUG ("Rx packet with size: " << pkt->GetSize ());
  Time now = Simulator::Now ();
  Time txTime = hdr.GetTs ();
  double pktDelay = now.GetSeconds () - txTime.GetSeconds ();
  NS_LOG_DEBUG ("Delay for packet with seq=" << hdr.GetSeq () << " is: " << pktDelay*1e3 << " ms");
  *stream2->GetStream () << Simulator::Now ().GetSeconds () << "\t" << pktDelay << std::endl;
}

static void
Sinr (uint64_t imsi, SpectrumValue& oldSinr, SpectrumValue& newSinr)
{
  NS_LOG_UNCOND (10 * log10 (Sum (newSinr)));
}

int
main (int argc, char *argv[])
{
  bool harqEnabled = true;
  bool rlcAmEnabled = true;
  uint32_t appPacketSize = 1440; // packet size at the application layer [bytes]
  uint16_t enbAntennaNum = 64; // number of antenna elements at the BS
  uint16_t ueAntennaNum = 16; // number of antenna elements at the UE
  double frequency = 28e9; // operating frequency [Hz]
  double txPow = 30.0; // tx power [dBm]
  double noiseFigure = 9.0; // noise figure [dB]
  double distance = 50.0; // distance between tx and rx nodes [m]
  uint32_t simTime = 10; // simulation time [s]
  uint32_t updatePeriod = 10; // channel/channel condition update period [ms]
  uint16_t nonSelfBlocking = 4;
  uint32_t remoteHostDelay = 10; // delay from PGW to remote host [ms]
  std::string outputFolder = "../scc-results/"; // path to the main output folder for the results
  std::string scenario = "UMi-StreetCanyon"; // 3GPP propagation scenario
  bool isBlockage = false;
  bool enableLog = false;

  CommandLine cmd;
  cmd.AddValue ("updatePeriod", "Channel/channel condition update periodicity [ms]", updatePeriod);
  cmd.AddValue ("blockage", "Enable blockage model A of the 3GPP channel model", isBlockage);
  cmd.AddValue ("nonSelfBlocking", "Number of non self-blocking components", nonSelfBlocking);
  cmd.AddValue ("distance", "Initial distance from the gNB [m]", distance);
  cmd.AddValue ("log", "Enable logging components", enableLog);
  cmd.Parse (argc, argv);

  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (1);

  std::string blockageOutput = "no-blockage-";
  if (isBlockage)
  {
    blockageOutput = "yes-blockage-";
  }

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
  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (updatePeriod)));
  Config::SetDefault ("ns3::ThreeGppChannelModel::PortraitMode", BooleanValue (false)); // blockage model with UT in landscape mode
  Config::SetDefault ("ns3::ThreeGppChannelConditionModel::UpdatePeriod", TimeValue (MilliSeconds (updatePeriod)));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue (100e6)); // fixed 100 MHz bandwidth
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (frequency));

  // antenna settings
  Config::SetDefault ("ns3::ThreeGppAntennaArrayModel::IsotropicElements", BooleanValue (true));

  Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (harqEnabled));

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
  enbNodes.Create (1);
  ueNodes.Create (1);

  // create mobility models
  Ptr<MobilityModel> enbMobility= CreateObject<ConstantPositionMobilityModel> ();
  enbMobility->SetPosition (Vector (0.0, 0.0, 10.0));
  Ptr<MobilityModel> ueMobility = CreateObject<RandomWalk2dOutdoorMobilityModel> (); // 2D random walk mobility model for the UE
  ueMobility->SetPosition (Vector (distance, 0.0, 1.6));
  ueMobility->SetAttribute ("Bounds", RectangleValue (Rectangle (-200.0, 200.0, -200.0, 200.0)));
  ueMobility->SetAttribute ("Mode", EnumValue (RandomWalk2dOutdoorMobilityModel::MODE_DISTANCE)); // updating mode for UE direction and speed
  ueMobility->SetAttribute ("Distance", DoubleValue (2.0)); // update UE direction and speed every VALUE meters walked

  // assign mobility models to the nodes
  enbNodes.Get (0)->AggregateObject (enbMobility);
  ueNodes.Get (0)->AggregateObject (ueMobility);

  // Create the tx and rx devices
  NetDeviceContainer enbMmWaveDevs = mmwaveHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueMmWaveDevs = mmwaveHelper->InstallUeDevice (ueNodes);

  // install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueMmWaveDevs)); // assign IP address to UEs
  Ptr<Node> ueNode = ueNodes.Get (0); // set the default gateway for the UE
  Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
  ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

  // attach UE to the Mmwave BS
  mmwaveHelper->AttachToClosestEnb (ueMmWaveDevs, enbMmWaveDevs);

  // create applications
  uint16_t ulPort = 2000;

  ApplicationContainer clientApps;
  ApplicationContainer serverApps;
  ++ulPort;
  PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
  ulPacketSinkHelper.SetAttribute ("EnableSeqTsSizeHeader", BooleanValue (true)); // enable SeqTs header to measure end-to-end delay
  serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

  OnOffHelper ulOnOffHelper ("ns3::UdpSocketFactory", InetSocketAddress (remoteHostAddr, ulPort));
  ulOnOffHelper.SetAttribute ("EnableSeqTsSizeHeader", BooleanValue (true)); // enable SeqTs header to measure end-to-end delay
  ulOnOffHelper.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=10000.0]"));
  ulOnOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
  ulOnOffHelper.SetAttribute ("DataRate", DataRateValue (DataRate ("100Mbps")));
  ulOnOffHelper.SetAttribute ("PacketSize", UintegerValue (appPacketSize));
  clientApps.Add (ulOnOffHelper.Install (ueNode));

  Time appStartTime = Seconds (0.01);
  serverApps.Start (appStartTime);
  clientApps.Start (appStartTime);

  // (OPTIONAL) enable output traces
  // mmwaveHelper->EnableTraces ();

  AsciiTraceHelper asciiTraceHelper;
  stream1 = asciiTraceHelper.CreateFileStream (outputFolder+blockageOutput+"thr-trace.csv");
  stream2 = asciiTraceHelper.CreateFileStream (outputFolder+blockageOutput+"delay-trace.csv");
  stream3 = asciiTraceHelper.CreateFileStream (outputFolder+blockageOutput+"snr-trace.csv");

  // connect to traces
  Ptr<MmWaveUeNetDevice> ueNetDev = StaticCast <MmWaveUeNetDevice> (ueMmWaveDevs.Get (0));
  Ptr<MmWaveUePhy> uePhy = ueNetDev->GetPhy ();
  uePhy->TraceConnectWithoutContext ("ReportCurrentCellRsrpSinr", MakeCallback (&Sinr));
  sinkApp = StaticCast<PacketSink> (serverApps.Get (0));
  sinkApp->TraceConnectWithoutContext ("RxWithSeqTsSize", MakeCallback (&Rx));
  Simulator::Schedule (appStartTime, &ComputeE2eThroughput);

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();

  // double sum = std::accumulate(std::begin(snrVsTime), std::end(snrVsTime), 0.0);
  // NS_LOG_UNCOND ("Average SNR over Time=" << sum / snrVsTime.size() << " dB");

  return 0;
}
