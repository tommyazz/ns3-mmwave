import sys
import ns.applications
import ns.core
import ns.internet
import ns.network
import ns.point_to_point
import ns.mmwave
import ns.mobility
import ns.buildings
import ns.spectrum
import ns
import math
import torch

totRx = 0
lastRxBytes = 0

def ComputeE2eThroughput(timeRes):
    global lastRxBytes
    totRxBytes = totRx
    rxBytes = totRxBytes - lastRxBytes
    lastRxBytes = totRxBytes
    thr = rxBytes * 8.0 / (timeRes * 1e-3) / 1e6  # Throughput in Mbps
    print(ns.core.Simulator.Now().GetSeconds(), thr, sep="---") 
    ns.core.Simulator.Schedule (ns.core.MilliSeconds (timeRes), ComputeE2eThroughput, timeRes)

def rx_callback(socket):
    global totRx
    fromAddr = ns.network.Address()
    packet = socket.RecvFrom(fromAddr)
    #print("rx bytes", packet.GetSize(), " from ", ns.network.InetSocketAddress.ConvertFrom(fromAddr).GetIpv4 ())
    totRx += packet.GetSize()
    #print(totRx)

def SetSocketRxCallback(app):
    rxSocket = app.GetListeningSocket()
    rxSocket.SetRecvCallback(rx_callback)


def torch_cuda():
	if torch.cuda.is_available():
		print(ns.core.Simulator.Now().GetSeconds())
		print("CUDA available")

	ns.core.Simulator.Schedule(ns.core.MilliSeconds(100), torch_cuda)

def main(argv):

    cmd = ns.core.CommandLine()
    cmd.harqEnabled = True
    cmd.rlcAmEnabled = True
    cmd.appPacketSize = 1440  # packet size at the application layer [bytes]
    cmd.enbAntennaNum = 64    # number of antenna elements at the BS
    cmd.ueAntennaNum = 16     # number of antenna elements at the UE
    cmd.txPower = 30.0        # tx power [dBm]
    cmd.noiseFigure = 9.0     # noise figure [dB]
    cmd.distance = 50.0       # distance between tx and rx nodes [m]
    cmd.simTime = 10          # simulation time [s]
    cmd.updatePeriod = 10     # channel/channel condition update period [ms]
    cmd.nonSelfBlocking = 4   # number of self-blocking components
    cmd.remoteHostDelay = 10  # delay from PGW to remote host [ms]
    cmd.outputFolder = "../scc-results/" # path to the main output folder for the results
    cmd.isBlockage = False
    cmd.enableLog = False
 
    ns.core.Config.SetDefault("ns3::OnOffApplication::PacketSize", ns.core.StringValue("1472"))
    ns.core.Config.SetDefault("ns3::OnOffApplication::DataRate", ns.core.StringValue("100kb/s"))
    
    cmd.AddValue ("updatePeriod", "Channel/channel condition update periodicity [ms]")
    cmd.AddValue ("blockage", "Enable blockage model A of the 3GPP channel model")
    cmd.AddValue ("nonSelfBlocking", "Number of non self-blocking components")
    cmd.AddValue ("distance", "Initial distance from the gNB [m]")
    cmd.AddValue ("log", "Enable logging components")

    cmd.Parse(argv)

    harqEnabled = bool(cmd.harqEnabled)
    rlcAmEnabled = bool(cmd.rlcAmEnabled) 
    appPacketSize = int(cmd.appPacketSize)
    enbAntennaNum = int(cmd.enbAntennaNum)
    ueAntennaNum = int(cmd.ueAntennaNum)
    txPower = float(cmd.txPower)
    noiseFigure = float(cmd.noiseFigure)
    distance = int(cmd.distance)
    simTime = int(cmd.simTime)
    updatePeriod = int(cmd.updatePeriod)
    nonSelfBlocking = int(cmd.nonSelfBlocking)
    remoteHostDelay = int(cmd.remoteHostDelay)
    outputFolder = str(cmd.outputFolder)
    isBlockage = bool(cmd.isBlockage)
    enableLog = bool(cmd.enableLog)

    frequency = 28e9                # operating frequency [Hz]
    scenario = "UMi-StreetCanyon"   # 3GPP propagation scenario
    timeRes = 10                    # time resolution for thr calculation [ms]

    ns.core.RngSeedManager.SetSeed (1);
    ns.core.RngSeedManager.SetRun (1);

    blockageOutput = "no-blockage-"
    if isBlockage:
        blockageOutput = "yes-blockage-"

    if enableLog:
        ns.core.LogComponentEnable("RandomWalk2dOutdoor", ns.core.LOG_LEVEL_ALL)
 
    # setting power and noise figure
    ns.core.Config.SetDefault ("ns3::MmWaveEnbPhy::TxPower", ns.core.DoubleValue (txPower))
    ns.core.Config.SetDefault ("ns3::MmWaveEnbPhy::NoiseFigure", ns.core.DoubleValue (noiseFigure))
    ns.core.Config.SetDefault ("ns3::MmWaveUePhy::TxPower", ns.core.DoubleValue (txPower))
    ns.core.Config.SetDefault ("ns3::MmWaveUePhy::NoiseFigure", ns.core.DoubleValue (noiseFigure))

    # setting the 3GPP channel model
    ns.core.Config.SetDefault ("ns3::ThreeGppChannelModel::Blockage", ns.core.BooleanValue (isBlockage))
    ns.core.Config.SetDefault ("ns3::ThreeGppChannelModel::NumNonselfBlocking", ns.core.IntegerValue (nonSelfBlocking))
    ns.core.Config.SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", ns.core.TimeValue (ns.core.MilliSeconds (updatePeriod)))
    ns.core.Config.SetDefault ("ns3::ThreeGppChannelModel::PortraitMode", ns.core.BooleanValue (False)) # blockage model with UT in landscape mode
    ns.core.Config.SetDefault ("ns3::ThreeGppChannelConditionModel::UpdatePeriod", ns.core.TimeValue (ns.core.MilliSeconds (updatePeriod)))
    ns.core.Config.SetDefault ("ns3::MmWavePhyMacCommon::Bandwidth", ns.core.DoubleValue (100e6)) # fixed 100 MHz bandwidth
    ns.core.Config.SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", ns.core.DoubleValue (frequency))

    # antenna settings
    ns.core.Config.SetDefault ("ns3::ThreeGppAntennaArrayModel::IsotropicElements", ns.core.BooleanValue (True))

    ns.core.Config.SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", ns.core.BooleanValue (rlcAmEnabled))
    ns.core.Config.SetDefault ("ns3::MmWaveHelper::HarqEnabled", ns.core.BooleanValue (harqEnabled))
    ns.core.Config.SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", ns.core.BooleanValue (harqEnabled))

    mmwaveHelper = ns.mmwave.mmwave.MmWaveHelper()
    mmwaveEpcHelper = ns.mmwave.mmwave.MmWavePointToPointEpcHelper()
    
    if scenario == "UMi-StreetCanyon":
        mmwaveHelper.SetPathlossModelType ("ns3::ThreeGppUmiStreetCanyonPropagationLossModel")
        mmwaveHelper.SetChannelConditionModelType ("ns3::ThreeGppUmiStreetCanyonChannelConditionModel")
    else:
        raise ValueError("Unkown scenario")

    # setting antenna parameters
    mmwaveHelper.SetMmWaveEnbNetDeviceAttribute ("AntennaNum", ns.core.UintegerValue (enbAntennaNum))
    mmwaveHelper.SetMmWaveUeNetDeviceAttribute ("AntennaNum", ns.core.UintegerValue (ueAntennaNum))
    # setting scheduler
    mmwaveHelper.SetSchedulerType ("ns3::MmWaveFlexTtiMacScheduler")

    mmwaveHelper.SetEpcHelper (mmwaveEpcHelper)
    mmwaveHelper.SetHarqEnabled (harqEnabled)
    mmwaveHelper.Initialize ()

    # get SGW/PGW and create the remote host
    pgw = mmwaveEpcHelper.GetPgwNode ()
    remoteHostContainer = ns.network.NodeContainer()
    remoteHostContainer.Create (1)
    remoteHost = remoteHostContainer.Get (0)
    internet = ns.internet.InternetStackHelper()
    internet.Install (remoteHostContainer)

    # connect remote host and PGW
    p2ph = ns.point_to_point.PointToPointHelper()
    p2ph.SetDeviceAttribute ("DataRate", ns.network.DataRateValue (ns.network.DataRate ("100Gb/s")))
    p2ph.SetDeviceAttribute ("Mtu", ns.core.UintegerValue (1500))
    p2ph.SetChannelAttribute ("Delay", ns.core.TimeValue (ns.core.MilliSeconds (remoteHostDelay)))
    internetDevices = p2ph.Install (pgw, remoteHost)
    ipv4h = ns.internet.Ipv4AddressHelper()
    ipv4h.SetBase (ns.network.Ipv4Address("1.0.0.0"), ns.network.Ipv4Mask("255.0.0.0"))
    internetIpIfaces = ipv4h.Assign (internetDevices)
    # interface 0 is localhost, 1 is the p2p device
    remoteHostAddr = internetIpIfaces.GetAddress (1)
    ipv4RoutingHelper = ns.internet.Ipv4StaticRoutingHelper()
    remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost.GetObject(ns.core.TypeId.LookupByName("ns3::Ipv4")));
    remoteHostStaticRouting.AddNetworkRouteTo (ns.network.Ipv4Address ("7.0.0.0"), ns.network.Ipv4Mask ("255.0.0.0"), 1);

    # create nodes
    enbNodes = ns.network.NodeContainer()
    ueNodes = ns.network.NodeContainer()
    enbNodes.Create (1)
    ueNodes.Create (1)

    # create mobility models
    enbMobility = ns.mobility.ConstantPositionMobilityModel ()
    enbMobility.SetPosition (ns.core.Vector (0.0, 0.0, 10.0))
    ueMobility = ns.buildings.RandomWalk2dOutdoorMobilityModel() # 2D random walk mobility model for the UE
    ueMobility.SetPosition (ns.core.Vector (distance, 0.0, 1.6))
    ueMobility.SetAttribute ("Bounds", ns.mobility.RectangleValue (ns.mobility.Rectangle (-200.0, 200.0, -200.0, 200.0)))
    ueMobility.SetAttribute ("Mode", ns.core.EnumValue (ns.buildings.RandomWalk2dOutdoorMobilityModel.MODE_DISTANCE)) # updating mode for the UE direction and speed
    ueMobility.SetAttribute ("Distance", ns.core.DoubleValue (2.0)) # update UE direction and speed every VALUE meters walked

    # assign mobility models to the nodes
    enbNodes.Get (0).AggregateObject (enbMobility)
    ueNodes.Get (0).AggregateObject (ueMobility)

    # Create the tx and rx devices
    enbMmWaveDevs = mmwaveHelper.InstallEnbDevice (enbNodes)
    ueMmWaveDevs = mmwaveHelper.InstallUeDevice (ueNodes)

    # install the IP stack on the UEs
    internet.Install (ueNodes)
    ueIpIface = ns.internet.Ipv4InterfaceContainer()
    ueIpIface = mmwaveEpcHelper.AssignUeIpv4Address (ueMmWaveDevs) # assign IP address to UEs
    ueNode = ueNodes.Get (0) # set the default gateway for the UE
    ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode.GetObject(ns.core.TypeId.LookupByName("ns3::Ipv4")))
    ueStaticRouting.SetDefaultRoute (mmwaveEpcHelper.GetUeDefaultGatewayAddress (), 1)

    # attach UE to the Mmwave BS
    mmwaveHelper.AttachToClosestEnb (ueMmWaveDevs, enbMmWaveDevs)
    print("UE attached to BS...")

    # create applications
    ulPort = 2000

    clientApps = ns.network.ApplicationContainer()
    serverApps = ns.network.ApplicationContainer()
    ulPort += 1 
    ulPacketSinkHelper = ns.applications.PacketSinkHelper("ns3::UdpSocketFactory", ns.network.InetSocketAddress (ns.network.Ipv4Address.GetAny (), ulPort))
    ulPacketSinkHelper.SetAttribute ("EnableSeqTsSizeHeader", ns.core.BooleanValue (True)) # enable SeqTs header to measure end-to-end delay
    serverApps.Add (ulPacketSinkHelper.Install (remoteHost))

    ulOnOffHelper = ns.applications.OnOffHelper("ns3::UdpSocketFactory", ns.network.InetSocketAddress (remoteHostAddr, ulPort))
    ulOnOffHelper.SetAttribute ("EnableSeqTsSizeHeader", ns.core.BooleanValue (True)) # enable SeqTs header to measure end-to-end delay
    ulOnOffHelper.SetAttribute ("OnTime", ns.core.StringValue ("ns3::ConstantRandomVariable[Constant=10000.0]"))
    ulOnOffHelper.SetAttribute ("OffTime", ns.core.StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"))
    ulOnOffHelper.SetAttribute ("DataRate", ns.network.DataRateValue (ns.network.DataRate ("100Mbps")))
    ulOnOffHelper.SetAttribute ("PacketSize", ns.core.UintegerValue (appPacketSize));
    clientApps.Add (ulOnOffHelper.Install (ueNode))
    print("Applications installed...")

    appStartTime = ns.core.Seconds (0.01)
    serverApps.Start (appStartTime)
    clientApps.Start (appStartTime)

    # (OPTIONAL) enable output traces
    # mmwaveHelper.EnableTraces ()

    #asciiTraceHelper = ns.network.AsciiTraceHelper()
    #stream1 = asciiTraceHelper.CreateFileStream (outputFolder+blockageOutput+"thr-trace.csv")
    #stream2 = asciiTraceHelper.CreateFileStream (outputFolder+blockageOutput+"delay-trace.csv")
    #stream3 = asciiTraceHelper.CreateFileStream (outputFolder+blockageOutput+"snr-trace.csv")

    # connect to traces
    ueNetDev = ueMmWaveDevs.Get(0).GetObject(ns.mmwave.mmwave.MmWaveUeNetDevice.GetTypeId())
    uePhy = ueNetDev.GetPhy ()
    # uePhy.TraceConnectWithoutContext ("ReportCurrentCellRsrpSinr", ns.core.CallbackValue(ns.core.CallbackBase("Sinr")));
    sinkApp = serverApps.Get (0).GetObject(ns.applications.PacketSink.GetTypeId())
    
    ns.core.Simulator.Schedule(appStartTime+ns.core.Seconds(0.00001), SetSocketRxCallback, sinkApp)
    ns.core.Simulator.Schedule(appStartTime, ComputeE2eThroughput, timeRes)
    ns.core.Simulator.Schedule(ns.core.Seconds(1.0), torch_cuda)

    ns.core.Simulator.Stop(ns.core.Seconds(2.0))
    ns.core.Simulator.Run()
    ns.core.Simulator.Destroy()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))