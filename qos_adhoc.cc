/* =============================================================================
 * QoS-Aware Adaptive Routing in Ad Hoc Networks using Traffic Prioritization
 * NS-3 Simulation — C++ Source File
 *
 * Project: Mini Project — Ad Hoc Wireless Network
 * Routing: AODV
 * MAC/PHY: IEEE 802.11 (Ad Hoc mode)
 * Traffic: 3 QoS classes (High/Medium/Low priority)
 * Metrics: FlowMonitor → output.xml
 *
 * Run:
 *   cp qos_adhoc.cc <ns3-root>/scratch/
 *   cd <ns3-root>
 *   ./ns3 run "scratch/qos_adhoc --nNodes=20 --simTime=60"
 * =============================================================================
 */

// ─── Standard NS-3 Core Headers ───────────────────────────────────────────────
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ipv4-flow-classifier.h"

// ─── Standard C++ Headers ─────────────────────────────────────────────────────
#include <iostream>
#include <fstream>
#include <string>
#include <map>

using namespace ns3;

// ─── NS-3 Logging Component ───────────────────────────────────────────────────
NS_LOG_COMPONENT_DEFINE("QoSAdHocSimulation");

// =============================================================================
// HELPER: Install a UDP OnOff application (High / Medium priority traffic)
// Parameters:
//   srcNode  — the sending node
//   dstAddr  — destination IP address
//   port     — UDP port number
//   interval — packet send interval (determines priority level)
//   pktSize  — payload size in bytes
//   startT   — application start time (seconds)
//   stopT    — application stop time (seconds)
// =============================================================================
void InstallUdpOnOff(Ptr<Node> srcNode,
                     Ipv4Address dstAddr,
                     uint16_t    port,
                     double      interval,
                     uint32_t    pktSize,
                     double      startT,
                     double      stopT)
{
    // Create a UDP sink on destination (always port = port)
    // Note: sinks are installed globally; this helper focuses on the sender.
    OnOffHelper onoff("ns3::UdpSocketFactory",
                      InetSocketAddress(dstAddr, port));

    // DataRate is derived from packet size and interval
    // e.g. 1000 bytes every 0.02 s ≈ 400 kbps
    double bitsPerSecond = (pktSize * 8.0) / interval;
    onoff.SetConstantRate(DataRate(static_cast<uint64_t>(bitsPerSecond)));
    onoff.SetAttribute("PacketSize", UintegerValue(pktSize));

    // OnTime = always on, OffTime = always off (constant stream)
    onoff.SetAttribute("OnTime",  StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

    ApplicationContainer app = onoff.Install(srcNode);
    app.Start(Seconds(startT));
    app.Stop(Seconds(stopT));
}

// =============================================================================
// HELPER: Install a TCP BulkSend application (Low priority — file transfer)
// =============================================================================
void InstallTcpBulkSend(Ptr<Node> srcNode,
                        Ipv4Address dstAddr,
                        uint16_t    port,
                        uint32_t    maxBytes,   // 0 = unlimited
                        double      startT,
                        double      stopT)
{
    BulkSendHelper bulk("ns3::TcpSocketFactory",
                        InetSocketAddress(dstAddr, port));
    bulk.SetAttribute("MaxBytes", UintegerValue(maxBytes));
    bulk.SetAttribute("SendSize", UintegerValue(1460)); // standard TCP MSS

    ApplicationContainer app = bulk.Install(srcNode);
    app.Start(Seconds(startT));
    app.Stop(Seconds(stopT));
}

// =============================================================================
// HELPER: Install a PacketSink on a node (receives UDP or TCP)
// =============================================================================
void InstallSink(Ptr<Node> node, std::string protocol, uint16_t port)
{
    PacketSinkHelper sink(protocol, InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer app = sink.Install(node);
    app.Start(Seconds(0.0));
    app.Stop(Seconds(200.0)); // stop well after sim ends
}

// =============================================================================
// MAIN SIMULATION ENTRY POINT
// =============================================================================
int main(int argc, char *argv[])
{
    // ─── Command-Line Arguments (Bonus Feature) ──────────────────────────────
    uint32_t nNodes  = 20;   // default: 20 wireless nodes
    double   simTime = 60.0; // default: 60 seconds

    CommandLine cmd(__FILE__);
    cmd.AddValue("nNodes",  "Total number of wireless ad hoc nodes", nNodes);
    cmd.AddValue("simTime", "Total simulation time in seconds",       simTime);
    cmd.Parse(argc, argv);

    // Enforce minimum node count for 3 traffic pairs + spare nodes
    if (nNodes < 6)
    {
        NS_LOG_ERROR("Need at least 6 nodes. Using 6.");
        nNodes = 6;
    }

    NS_LOG_UNCOND("=== QoS Ad Hoc Simulation ===");
    NS_LOG_UNCOND("Nodes: " << nNodes << " | SimTime: " << simTime << "s");

    // ─── Enable Verbose Logging (comment out to reduce output) ───────────────
    // LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_INFO);
    // LogComponentEnable("OnOffApplication",    LOG_LEVEL_INFO);

    // =========================================================================
    // SECTION 1: NODE CREATION
    // Create a container of wireless nodes representing mobile devices
    // =========================================================================
    NS_LOG_UNCOND("[1] Creating " << nNodes << " wireless nodes...");
    NodeContainer nodes;
    nodes.Create(nNodes);

    // =========================================================================
    // SECTION 2: WIFI PHY + MAC (IEEE 802.11 Ad Hoc Mode)
    // Configure the wireless channel, physical layer, and MAC layer
    // =========================================================================
    NS_LOG_UNCOND("[2] Setting up WiFi (IEEE 802.11 Ad Hoc)...");

    // WiFi channel model: log-distance propagation loss + constant speed prop.
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                   "Exponent",      DoubleValue(2.5),
                                   "ReferenceDistance", DoubleValue(1.0),
                                   "ReferenceLoss", DoubleValue(46.68));

    // Physical layer helper
    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel.Create());
    wifiPhy.Set("TxPowerStart", DoubleValue(20.0)); // dBm
    wifiPhy.Set("TxPowerEnd",   DoubleValue(20.0));

    // MAC layer: Ad Hoc mode (no Access Point)
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    // WiFi standard: 802.11b (widely compatible, good for ad hoc)
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);
    // Remote station manager: constant-rate at 11 Mbps
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",    StringValue("DsssRate11Mbps"),
                                 "ControlMode", StringValue("DsssRate1Mbps"));

    // Install WiFi devices on all nodes
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, nodes);

    // =========================================================================
    // SECTION 3: MOBILITY MODEL (Random Waypoint)
    // Nodes move randomly within a 500x500m area, simulating mobile devices
    // =========================================================================
    NS_LOG_UNCOND("[3] Configuring RandomWaypoint mobility...");

    MobilityHelper mobility;

    // Rectangular bounding box for node movement
    mobility.SetPositionAllocator(
        "ns3::RandomRectanglePositionAllocator",
        "X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"),
        "Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));

    // RandomWaypointMobilityModel: nodes pick random destinations and move
    // Speed: 1–15 m/s (walking to fast vehicle)
    // Pause: 0–2 s at each waypoint
    mobility.SetMobilityModel(
        "ns3::RandomWaypointMobilityModel",
        "Speed",  StringValue("ns3::UniformRandomVariable[Min=1.0|Max=15.0]"),
        "Pause",  StringValue("ns3::ConstantRandomVariable[Constant=1.0]"),
        "PositionAllocator",
            StringValue("ns3::RandomRectanglePositionAllocator"));

    mobility.Install(nodes);

    // =========================================================================
    // SECTION 4: ROUTING PROTOCOL (AODV)
    // AODV: Ad hoc On-Demand Distance Vector routing
    // Discovers routes dynamically when traffic demands it
    // =========================================================================
    NS_LOG_UNCOND("[4] Installing AODV routing protocol...");

    AodvHelper aodv;
    // Tune AODV parameters for better performance
    aodv.Set("ActiveRouteTimeout",  TimeValue(Seconds(15.0)));
    aodv.Set("NetDiameter",         UintegerValue(35));
    aodv.Set("HelloInterval",       TimeValue(Seconds(1.0)));
    aodv.Set("RreqRetries",         UintegerValue(3));

    // Install Internet stack (IP + TCP/UDP + AODV)
    InternetStackHelper internet;
    internet.SetRoutingHelper(aodv);
    internet.Install(nodes);

    // Assign IP addresses (10.1.1.x subnet)
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

    // =========================================================================
    // SECTION 5: QoS TRAFFIC SETUP
    //
    // QoS Logic: Three priority classes differentiated by:
    //   • Packet interval (lower interval = higher effective bandwidth)
    //   • Packet size (tuned per traffic type)
    //   • Protocol (UDP for real-time; TCP for file transfer)
    //
    // CLASS A — HIGH PRIORITY (Video / Real-time)
    //   UDP, interval=0.02s, pktSize=1024B → ~409 kbps per flow
    //   Nodes: 0→1, 2→3
    //
    // CLASS B — MEDIUM PRIORITY (VoIP / Interactive)
    //   UDP, interval=0.08s, pktSize=512B → ~51 kbps per flow
    //   Nodes: 4→5, 6→7
    //
    // CLASS C — LOW PRIORITY (File Transfer / Background)
    //   TCP BulkSend → fills available bandwidth greedily
    //   Nodes: 8→9, 10→11
    // =========================================================================
    NS_LOG_UNCOND("[5] Installing QoS traffic applications...");

    // ── PORT ASSIGNMENTS ─────────────────────────────────────────────────────
    const uint16_t HIGH_PORT_1   = 5001;
    const uint16_t HIGH_PORT_2   = 5002;
    const uint16_t MED_PORT_1    = 6001;
    const uint16_t MED_PORT_2    = 6002;
    const uint16_t LOW_PORT_1    = 7001;
    const uint16_t LOW_PORT_2    = 7002;

    // ── CLASS A: HIGH PRIORITY — UDP Real-Time (Video Streaming) ─────────────
    // QoS: Very small interval → low latency, high rate
    // Sink on node 1
    InstallSink(nodes.Get(1), "ns3::UdpSocketFactory", HIGH_PORT_1);
    InstallUdpOnOff(nodes.Get(0),
                    interfaces.GetAddress(1),
                    HIGH_PORT_1,
                    0.02,   // interval: 0.02s → ~409 kbps
                    1024,   // packet size: 1024 bytes
                    1.0,    // start: 1s (after AODV warms up)
                    simTime - 1.0);

    // Sink on node 3
    InstallSink(nodes.Get(3), "ns3::UdpSocketFactory", HIGH_PORT_2);
    InstallUdpOnOff(nodes.Get(2),
                    interfaces.GetAddress(3),
                    HIGH_PORT_2,
                    0.02,
                    1024,
                    1.0,
                    simTime - 1.0);

    // ── CLASS B: MEDIUM PRIORITY — UDP Interactive (VoIP / Gaming) ───────────
    // QoS: Moderate interval → balanced throughput/delay
    // Sink on node 5
    InstallSink(nodes.Get(5), "ns3::UdpSocketFactory", MED_PORT_1);
    InstallUdpOnOff(nodes.Get(4),
                    interfaces.GetAddress(5),
                    MED_PORT_1,
                    0.08,   // interval: 0.08s → ~51 kbps
                    512,    // packet size: 512 bytes
                    2.0,
                    simTime - 1.0);

    // Sink on node 7
    InstallSink(nodes.Get(7), "ns3::UdpSocketFactory", MED_PORT_2);
    InstallUdpOnOff(nodes.Get(6),
                    interfaces.GetAddress(7),
                    MED_PORT_2,
                    0.08,
                    512,
                    2.0,
                    simTime - 1.0);

    // ── CLASS C: LOW PRIORITY — TCP Bulk (File Transfer / FTP) ───────────────
    // QoS: TCP fills leftover bandwidth; higher inherent delay acceptable
    // Sink on node 9
    InstallSink(nodes.Get(9), "ns3::TcpSocketFactory", LOW_PORT_1);
    InstallTcpBulkSend(nodes.Get(8),
                       interfaces.GetAddress(9),
                       LOW_PORT_1,
                       0,        // unlimited bytes (runs until simTime)
                       3.0,
                       simTime - 1.0);

    // Sink on node 11
    InstallSink(nodes.Get(11), "ns3::TcpSocketFactory", LOW_PORT_2);
    InstallTcpBulkSend(nodes.Get(10),
                       interfaces.GetAddress(11),
                       LOW_PORT_2,
                       0,
                       3.0,
                       simTime - 1.0);

    // =========================================================================
    // SECTION 6: FLOW MONITOR — METRICS COLLECTION
    // FlowMonitor intercepts all packets and records:
    //   • txPackets/rxPackets → PDR
    //   • delaySum / rxPackets → mean end-to-end delay
    //   • rxBytes * 8 / duration → throughput
    // =========================================================================
    NS_LOG_UNCOND("[6] Installing FlowMonitor...");

    FlowMonitorHelper flowMonHelper;
    Ptr<FlowMonitor> flowMonitor = flowMonHelper.InstallAll();

    // =========================================================================
    // RUN SIMULATION
    // =========================================================================
    NS_LOG_UNCOND("[7] Running simulation for " << simTime << " seconds...");
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // =========================================================================
    // SECTION 7: POST-SIMULATION METRICS ANALYSIS
    // Print per-flow statistics to console and export to XML
    // =========================================================================
    NS_LOG_UNCOND("\n========== FLOW MONITOR RESULTS ==========");

    // Force FlowMonitor to check for lost packets before reporting
    flowMonitor->CheckForLostPackets();

    // Retrieve the flow classifier to map FlowId → 5-tuple
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowMonHelper.GetClassifier());

    // Get per-flow statistics map
    std::map<FlowId, FlowMonitor::FlowStats> stats = flowMonitor->GetFlowStats();

    // Counters for summary
    uint32_t totalTx = 0, totalRx = 0;
    double   totalDelay = 0.0, totalThroughput = 0.0;

    for (auto &entry : stats)
    {
        FlowId               flowId = entry.first;
        FlowMonitor::FlowStats fs   = entry.second;

        // Get 5-tuple for this flow
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);

        // Compute metrics
        double duration = fs.timeLastRxPacket.GetSeconds()
                        - fs.timeFirstTxPacket.GetSeconds();

        double throughputKbps = 0.0;
        if (duration > 0)
            throughputKbps = (fs.rxBytes * 8.0) / (duration * 1000.0);

        double meanDelayMs = 0.0;
        if (fs.rxPackets > 0)
            meanDelayMs = (fs.delaySum.GetSeconds() / fs.rxPackets) * 1000.0;

        double pdr = 0.0;
        if (fs.txPackets > 0)
            pdr = (double)fs.rxPackets / (double)fs.txPackets * 100.0;

        // Classify flow by destination port
        std::string trafficClass = "Unknown";
        uint16_t dport = t.destinationPort;
        if      (dport == HIGH_PORT_1 || dport == HIGH_PORT_2) trafficClass = "HIGH (UDP Video)";
        else if (dport == MED_PORT_1  || dport == MED_PORT_2)  trafficClass = "MEDIUM (UDP VoIP)";
        else if (dport == LOW_PORT_1  || dport == LOW_PORT_2)  trafficClass = "LOW (TCP File)";

        std::cout << "--------------------------------------------\n"
                  << "Flow " << flowId << " [" << trafficClass << "]\n"
                  << "  Src: " << t.sourceAddress      << ":" << t.sourcePort      << "\n"
                  << "  Dst: " << t.destinationAddress << ":" << t.destinationPort << "\n"
                  << "  Protocol: " << (t.protocol == 6 ? "TCP" : "UDP") << "\n"
                  << "  Tx Packets : " << fs.txPackets    << "\n"
                  << "  Rx Packets : " << fs.rxPackets    << "\n"
                  << "  Lost Pkts  : " << fs.lostPackets  << "\n"
                  << "  PDR        : " << pdr             << " %\n"
                  << "  Mean Delay : " << meanDelayMs     << " ms\n"
                  << "  Throughput : " << throughputKbps  << " kbps\n";

        totalTx          += fs.txPackets;
        totalRx          += fs.rxPackets;
        totalDelay       += meanDelayMs;
        totalThroughput  += throughputKbps;
    }

    std::cout << "============================================\n"
              << "SUMMARY\n"
              << "  Total Flows     : " << stats.size()  << "\n"
              << "  Total Tx Pkts   : " << totalTx       << "\n"
              << "  Total Rx Pkts   : " << totalRx       << "\n"
              << "  Overall PDR     : "
              << (totalTx > 0 ? (double)totalRx / totalTx * 100.0 : 0.0) << " %\n"
              << "  Avg Delay       : "
              << (stats.size() > 0 ? totalDelay / stats.size() : 0.0) << " ms\n"
              << "  Total Throughput: " << totalThroughput << " kbps\n"
              << "============================================\n";

    // ─── Export FlowMonitor Statistics to XML ────────────────────────────────
    flowMonitor->SerializeToXmlFile("output.xml", true, true);
    NS_LOG_UNCOND("[8] Results saved to output.xml");

    // ─── Cleanup ─────────────────────────────────────────────────────────────
    Simulator::Destroy();
    NS_LOG_UNCOND("Simulation complete.");

    return 0;
}
