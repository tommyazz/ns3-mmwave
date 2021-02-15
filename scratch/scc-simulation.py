import ns.applications
import ns.core
import ns.internet
import ns.network
import ns.point_to_point
import ns.mmwave
import ns.lte

lteHelper = ns.lte.LteHelper()

pointToPoint = ns.point_to_point.PointToPointHelper()

mmWaveEpcHelper = ns.mmwave.mmwave.MmWavePointToPointEpcHelper()
mmWaveHelper = ns.mmwave.mmwave.MmWaveHelper()
mmWaveHelper.SetPathlossModelType ("ns3::ThreeGppUmiStreetCanyonPropagationLossModel");

txPow = 30.0
ns.core.Config.SetDefault("ns3::MmWaveEnbPhy::TxPower", ns.core.DoubleValue(txPow))