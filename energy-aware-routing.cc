/*
 * Energy-aware routing for a mobile UAV swarm (ns-3 scratch simulation).
 *
 * EnergyAwareGlobalRouting periodically rebuilds a connectivity graph from
 * current UAV positions and residual energy, runs Dijkstra, and installs
 * IPv4 host routes. Baseline mode (--energyAware=false) uses distance only.
 * Proposed mode (--energyAware=true) adds an energy penalty and treats
 * relays below a global critical energy (criticalFraction * nominalEnergy)
 * as unusable.
 *
 *   ./ns3 run "scratch/energy-aware-routing --energyAware=true"
 *   ./ns3 run "scratch/energy-aware-routing --energyAware=false"
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;
using namespace ns3::energy;

NS_LOG_COMPONENT_DEFINE("EnergyAwareUavRouting");

// ---------------------------------------------------------------------------
// Simulation parameters (override from the command line)
// ---------------------------------------------------------------------------
static uint32_t g_numDrones = 12;
static double g_simTime = 200.0;
static double g_wifiRangeM = 90.0;
static double g_recomputeInterval = 2.0;
static double g_nominalEnergyJ = 500.0;   // full-pack UAV; used in cost + critical J
static double g_criticalFraction = 0.15;  // critical energy = fraction * nominal
static double g_energyWeight = 6.0;
static double g_distWeight = 1.0;
static bool g_energyAware = true;
static uint32_t g_rngSeed = 1;
static uint32_t g_rngRun = 1;
static std::string g_outPrefix = "";
static std::string g_energyTraceFile = "energy-trace.csv";
static std::string g_flowMonFile = "flow-results.xml";
static std::string g_routingEventsFile = "routing-events.csv";
static std::string g_metricsFile = "metrics.csv";
static std::string g_flowStatsFile = "flow-stats.csv";

static std::vector<double> g_remainingEnergy;
static std::vector<double> g_initialEnergyPerNode;
static std::vector<double> g_timeToCriticalS; // -1 = never crossed
static std::vector<Ptr<BasicEnergySource>> g_sources;
static bool g_firstDeathLogged = false;
static bool g_halfDeathLogged = false;
static std::ofstream g_energyTraceOut;
static std::ofstream g_routingEventsOut;
static uint32_t g_routingEventCount = 0;
static const uint32_t g_maxRoutingEvents = 20000;

static std::string
WithPrefix(const std::string& name)
{
    return g_outPrefix + name;
}

static double
CriticalEnergyJ()
{
    return g_criticalFraction * g_nominalEnergyJ;
}

static bool
IsCritical(double remainingJ)
{
    return remainingJ <= CriticalEnergyJ();
}

void
RemainingEnergyCallback(uint32_t nodeId, double /*oldValue*/, double newValue)
{
    g_remainingEnergy[nodeId] = newValue;

    if (g_timeToCriticalS[nodeId] < 0.0 && IsCritical(newValue))
    {
        g_timeToCriticalS[nodeId] = Simulator::Now().GetSeconds();
        NS_LOG_UNCOND("[CRITICAL] UAV " << nodeId << " crossed " << CriticalEnergyJ()
                                        << " J at t=" << g_timeToCriticalS[nodeId] << " s");
    }

    if (!g_firstDeathLogged && newValue <= 0.0)
    {
        g_firstDeathLogged = true;
        NS_LOG_UNCOND("[LIFETIME] First UAV (node " << nodeId << ") reached 0 J at t="
                                                    << Simulator::Now().GetSeconds() << " s");
    }

    if (!g_halfDeathLogged)
    {
        uint32_t dead = 0;
        for (double e : g_remainingEnergy)
        {
            if (e <= 0.0)
            {
                dead++;
            }
        }
        if (dead >= g_remainingEnergy.size() / 2)
        {
            g_halfDeathLogged = true;
            NS_LOG_UNCOND("[LIFETIME] 50% of swarm at 0 J at t="
                          << Simulator::Now().GetSeconds() << " s");
        }
    }
}

void
LogEnergyTrace()
{
    for (uint32_t i = 0; i < g_sources.size(); ++i)
    {
        double e = g_sources[i]->GetRemainingEnergy();
        g_remainingEnergy[i] = e;
        if (g_timeToCriticalS[i] < 0.0 && IsCritical(e))
        {
            g_timeToCriticalS[i] = Simulator::Now().GetSeconds();
            NS_LOG_UNCOND("[CRITICAL] UAV " << i << " crossed " << CriticalEnergyJ()
                                            << " J at t=" << g_timeToCriticalS[i] << " s");
        }
        g_energyTraceOut << Simulator::Now().GetSeconds() << "," << i << "," << e << "\n";
    }
    Simulator::Schedule(Seconds(1.0), &LogEnergyTrace);
}

/**
 * Centralized route manager: positions + residual energy -> Dijkstra ->
 * Ipv4StaticRouting host routes. Data-plane forwarding remains standard IPv4.
 */
class EnergyAwareGlobalRouting
{
  public:
    EnergyAwareGlobalRouting(NodeContainer nodes, Ipv4InterfaceContainer ifaces)
        : m_nodes(nodes),
          m_ifaces(ifaces)
    {
        m_n = nodes.GetN();
        m_lastNextHop.assign(m_n, std::vector<int>(m_n, -2));
    }

    void Start()
    {
        Simulator::Schedule(Seconds(0.01), &EnergyAwareGlobalRouting::Recompute, this);
    }

  private:
    NodeContainer m_nodes;
    Ipv4InterfaceContainer m_ifaces;
    uint32_t m_n;
    std::vector<std::vector<int>> m_lastNextHop;

    double Distance(uint32_t a, uint32_t b) const
    {
        Ptr<MobilityModel> ma = m_nodes.Get(a)->GetObject<MobilityModel>();
        Ptr<MobilityModel> mb = m_nodes.Get(b)->GetObject<MobilityModel>();
        return ma->GetDistanceFrom(mb);
    }

    /**
     * Link cost for using v as the next hop after u (distance u->v).
     *
     * Baseline:  cost = 1 + distWeight * (d / R)
     * Proposed:  cost = 1 + distWeight * (d / R) + energyWeight * (1 - E_v / E_nom)
     *            if E_v <= criticalFraction * E_nom then cost = 1e9
     *
     * Residual energy is compared to the swarm nominal pack (not each UAV's
     * own smaller battery), so a UAV that started with less energy is
     * expensive as a relay even while its own pack is still relatively full.
     */
    double RelayCost(uint32_t relay, double linkDistance) const
    {
        double distTerm = g_distWeight * (linkDistance / g_wifiRangeM);

        if (!g_energyAware)
        {
            return 1.0 + distTerm;
        }

        double e = g_remainingEnergy[relay];
        if (IsCritical(e))
        {
            return 1e9;
        }

        double fracUsed = 1.0 - (e / g_nominalEnergyJ);
        if (fracUsed < 0.0)
        {
            fracUsed = 0.0;
        }
        return 1.0 + distTerm + g_energyWeight * fracUsed;
    }

    std::vector<int> Dijkstra(uint32_t src) const
    {
        std::vector<double> dist(m_n, std::numeric_limits<double>::infinity());
        std::vector<int> pred(m_n, -1);
        std::vector<bool> visited(m_n, false);
        dist[src] = 0.0;

        using PQItem = std::pair<double, uint32_t>;
        std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;
        pq.push({0.0, src});

        while (!pq.empty())
        {
            auto [d, u] = pq.top();
            pq.pop();
            if (visited[u])
            {
                continue;
            }
            visited[u] = true;

            for (uint32_t v = 0; v < m_n; ++v)
            {
                if (v == u)
                {
                    continue;
                }
                double dxy = Distance(u, v);
                if (dxy > g_wifiRangeM)
                {
                    continue;
                }
                double w = RelayCost(v, dxy);
                if (dist[u] + w < dist[v])
                {
                    dist[v] = dist[u] + w;
                    pred[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }
        return pred;
    }

    int NextHopFrom(uint32_t src, uint32_t dest, const std::vector<int>& pred) const
    {
        if (pred[dest] == -1)
        {
            return -1;
        }
        int cur = dest;
        while (pred[cur] != static_cast<int>(src))
        {
            if (pred[cur] == -1)
            {
                return -1;
            }
            cur = pred[cur];
        }
        return cur;
    }

    void MaybeLogRouteChange(uint32_t s, uint32_t d, int nh, double hopCost)
    {
        if (m_lastNextHop[s][d] == nh)
        {
            return;
        }
        m_lastNextHop[s][d] = nh;
        if (!g_routingEventsOut.is_open() || g_routingEventCount >= g_maxRoutingEvents)
        {
            return;
        }
        double nhEnergy = (nh >= 0) ? g_remainingEnergy[nh] : -1.0;
        int nhCritical = (nh >= 0 && IsCritical(nhEnergy)) ? 1 : 0;
        g_routingEventsOut << Simulator::Now().GetSeconds() << "," << s << "," << d << "," << nh
                           << "," << (g_energyAware ? 1 : 0) << "," << nhEnergy << ","
                           << nhCritical << "," << hopCost << "\n";
        g_routingEventCount++;
    }

    void Recompute()
    {
        Ipv4StaticRoutingHelper staticHelper;

        for (uint32_t s = 0; s < m_n; ++s)
        {
            if (g_remainingEnergy[s] <= 0.0)
            {
                continue;
            }

            std::vector<int> pred = Dijkstra(s);
            Ptr<Ipv4> ipv4 = m_nodes.Get(s)->GetObject<Ipv4>();
            Ptr<Ipv4StaticRouting> table = staticHelper.GetStaticRouting(ipv4);

            // Remove previously installed host routes only. Deleting the
            // connected 10.1.1.0/24 route would break ARP for next hops.
            for (uint32_t r = table->GetNRoutes(); r > 0; --r)
            {
                Ipv4RoutingTableEntry entry = table->GetRoute(r - 1);
                if (entry.GetInterface() == 1 && entry.IsHost())
                {
                    table->RemoveRoute(r - 1);
                }
            }

            for (uint32_t d = 0; d < m_n; ++d)
            {
                if (d == s)
                {
                    continue;
                }
                int nh = NextHopFrom(s, d, pred);
                double hopCost = -1.0;
                if (nh >= 0)
                {
                    hopCost = RelayCost(nh, Distance(s, nh));
                    Ipv4Address destAddr = m_ifaces.GetAddress(d);
                    Ipv4Address nextHopAddr = m_ifaces.GetAddress(nh);
                    table->AddHostRouteTo(destAddr, nextHopAddr, 1);
                }
                MaybeLogRouteChange(s, d, nh, hopCost);
            }
        }

        Simulator::Schedule(Seconds(g_recomputeInterval),
                            &EnergyAwareGlobalRouting::Recompute,
                            this);
    }
};

static double
InitialEnergyForNode(uint32_t i)
{
    // Heterogeneous packs: these UAVs start closer to the critical joule
    // threshold so energy-aware Dijkstra has a real relay choice.
    // Values are above idle drain over simTime so a node that is not used as
    // a relay can finish above the critical line.
    // Sized so idle drain over simTime does not by itself force a UAV
    // below the global critical energy, but extra relay TX/RX can.
    if (i == 1)
    {
        return 160.0;
    }
    if (i == 5)
    {
        return 175.0;
    }
    if (i == 6)
    {
        return 190.0;
    }
    return g_nominalEnergyJ;
}

static void
WriteMetrics(Ptr<FlowMonitor> monitor, Ptr<Ipv4FlowClassifier> classifier)
{
    uint32_t nCritical = 0;
    double minE = std::numeric_limits<double>::infinity();
    double sumE = 0.0;
    double firstCritical = std::numeric_limits<double>::infinity();

    for (uint32_t i = 0; i < g_numDrones; ++i)
    {
        double e = g_remainingEnergy[i];
        sumE += e;
        minE = std::min(minE, e);
        if (IsCritical(e))
        {
            nCritical++;
        }
        if (g_timeToCriticalS[i] >= 0.0)
        {
            firstCritical = std::min(firstCritical, g_timeToCriticalS[i]);
        }
    }
    double avgE = sumE / g_numDrones;

    uint64_t txPackets = 0;
    uint64_t rxPackets = 0;
    uint64_t txBytes = 0;
    uint64_t rxBytes = 0;
    double delaySumS = 0.0;

    std::ofstream flowCsv(WithPrefix(g_flowStatsFile).c_str());
    flowCsv << "flowId,srcAddr,dstAddr,protocol,txPackets,rxPackets,lostPackets,txBytes,rxBytes,"
               "meanDelayS\n";

    const FlowMonitor::FlowStatsContainer& stats = monitor->GetFlowStats();
    for (auto const& kv : stats)
    {
        uint32_t flowId = kv.first;
        FlowMonitor::FlowStats fs = kv.second;
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
        if (t.protocol != 17) // UDP only (application CBR)
        {
            continue;
        }
        txPackets += fs.txPackets;
        rxPackets += fs.rxPackets;
        txBytes += fs.txBytes;
        rxBytes += fs.rxBytes;
        double meanDelay = (fs.rxPackets > 0) ? fs.delaySum.GetSeconds() / fs.rxPackets : 0.0;
        delaySumS += fs.delaySum.GetSeconds();
        std::ostringstream src;
        src << t.sourceAddress;
        std::ostringstream dst;
        dst << t.destinationAddress;
        flowCsv << flowId << "," << src.str() << "," << dst.str() << "," << int(t.protocol) << ","
                << fs.txPackets << "," << fs.rxPackets << "," << fs.lostPackets << ","
                << fs.txBytes << "," << fs.rxBytes << "," << meanDelay << "\n";
    }
    flowCsv.close();

    double pdr = (txPackets > 0) ? static_cast<double>(rxPackets) / static_cast<double>(txPackets)
                                 : 0.0;
    double throughputBps = (g_simTime > 0.0) ? (static_cast<double>(rxBytes) * 8.0 / g_simTime) : 0.0;
    double avgDelayS = (rxPackets > 0) ? delaySumS / static_cast<double>(rxPackets) : 0.0;

    std::ofstream m(WithPrefix(g_metricsFile).c_str());
    m << "metric,value\n";
    m << "energyAware," << (g_energyAware ? 1 : 0) << "\n";
    m << "simTimeS," << g_simTime << "\n";
    m << "numDrones," << g_numDrones << "\n";
    m << "nominalEnergyJ," << g_nominalEnergyJ << "\n";
    m << "criticalFraction," << g_criticalFraction << "\n";
    m << "criticalEnergyJ," << CriticalEnergyJ() << "\n";
    m << "energyWeight," << g_energyWeight << "\n";
    m << "minRemainingEnergyJ," << minE << "\n";
    m << "avgRemainingEnergyJ," << avgE << "\n";
    m << "totalRemainingEnergyJ," << sumE << "\n";
    m << "uavsBelowCritical," << nCritical << "\n";
    if (firstCritical < std::numeric_limits<double>::infinity())
    {
        m << "timeToFirstCriticalS," << firstCritical << "\n";
    }
    else
    {
        m << "timeToFirstCriticalS,NA\n";
    }
    m << "txPackets," << txPackets << "\n";
    m << "rxPackets," << rxPackets << "\n";
    m << "pdr," << pdr << "\n";
    m << "throughputBps," << throughputBps << "\n";
    m << "avgDelayS," << avgDelayS << "\n";

    for (uint32_t i = 0; i < g_numDrones; ++i)
    {
        m << "finalEnergyUav" << i << "J," << g_remainingEnergy[i] << "\n";
        m << "consumedEnergyUav" << i << "J,"
          << (g_initialEnergyPerNode[i] - g_remainingEnergy[i]) << "\n";
        m << "initialEnergyUav" << i << "J," << g_initialEnergyPerNode[i] << "\n";
        if (g_timeToCriticalS[i] >= 0.0)
        {
            m << "timeToCriticalUav" << i << "S," << g_timeToCriticalS[i] << "\n";
        }
        else
        {
            m << "timeToCriticalUav" << i << "S,NA\n";
        }
    }
    m.close();

    NS_LOG_UNCOND("========== SUMMARY ==========");
    NS_LOG_UNCOND("energyAware=" << (g_energyAware ? "true" : "false"));
    NS_LOG_UNCOND("criticalEnergyJ=" << CriticalEnergyJ());
    NS_LOG_UNCOND("minRemainingJ=" << minE << " avgRemainingJ=" << avgE
                                   << " totalRemainingJ=" << sumE);
    NS_LOG_UNCOND("uavsBelowCritical=" << nCritical);
    if (firstCritical < std::numeric_limits<double>::infinity())
    {
        NS_LOG_UNCOND("timeToFirstCriticalS=" << firstCritical);
    }
    else
    {
        NS_LOG_UNCOND("timeToFirstCriticalS=none (no UAV crossed critical)");
    }
    NS_LOG_UNCOND("PDR=" << pdr << " tx=" << txPackets << " rx=" << rxPackets);
    NS_LOG_UNCOND("throughputBps=" << throughputBps << " avgDelayS=" << avgDelayS);
    for (uint32_t i = 0; i < g_numDrones; ++i)
    {
        NS_LOG_UNCOND("  UAV " << i << " initial=" << g_initialEnergyPerNode[i]
                               << " J remaining=" << g_remainingEnergy[i] << " J");
    }
    NS_LOG_UNCOND("=============================");
}

int
main(int argc, char* argv[])
{
    CommandLine cmd;
    cmd.AddValue("numDrones", "Number of UAVs", g_numDrones);
    cmd.AddValue("simTime", "Simulation time (s)", g_simTime);
    cmd.AddValue("wifiRange", "Graph edge cutoff (m)", g_wifiRangeM);
    cmd.AddValue("recomputeInterval", "Route recompute period (s)", g_recomputeInterval);
    cmd.AddValue("initialEnergy", "Nominal / full-pack battery (J)", g_nominalEnergyJ);
    cmd.AddValue("criticalFraction", "Critical energy as fraction of nominal pack", g_criticalFraction);
    cmd.AddValue("energyWeight", "Weight of residual-energy term in path cost", g_energyWeight);
    cmd.AddValue("distWeight", "Weight of distance term in path cost", g_distWeight);
    cmd.AddValue("energyAware", "true = proposed energy-aware routing", g_energyAware);
    cmd.AddValue("rngSeed", "RngSeedManager seed", g_rngSeed);
    cmd.AddValue("rngRun", "RngSeedManager run number", g_rngRun);
    cmd.AddValue("outPrefix", "Directory prefix for output files (e.g. results/baseline/)", g_outPrefix);
    cmd.AddValue("energyTraceFile", "Energy CSV filename (after prefix)", g_energyTraceFile);
    cmd.AddValue("flowMonFile", "FlowMonitor XML filename (after prefix)", g_flowMonFile);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(g_rngSeed);
    RngSeedManager::SetRun(g_rngRun);

    g_remainingEnergy.assign(g_numDrones, g_nominalEnergyJ);
    g_initialEnergyPerNode.assign(g_numDrones, g_nominalEnergyJ);
    g_timeToCriticalS.assign(g_numDrones, -1.0);

    g_energyTraceOut.open(WithPrefix(g_energyTraceFile).c_str());
    g_energyTraceOut << "time,nodeId,remainingEnergyJ\n";
    g_routingEventsOut.open(WithPrefix(g_routingEventsFile).c_str());
    g_routingEventsOut << "time,source,destination,nextHop,energyAware,nextHopEnergy,"
                          "nextHopCritical,routeCost\n";

    NS_LOG_UNCOND("Running energyAware=" << (g_energyAware ? "true" : "false")
                                         << " numDrones=" << g_numDrones
                                         << " seed=" << g_rngSeed << " run=" << g_rngRun
                                         << " criticalJ=" << CriticalEnergyJ());

    NodeContainer drones;
    drones.Create(g_numDrones);

    // 4 x 3 grid, 50 m spacing: well inside 802.11g receive range at 22 dBm.
    // Graph cutoff (wifiRange) is 90 m so 4-neighbours and diagonals are
    // edges, while far pairs still need a relay.
    double delta = 50.0;
    uint32_t gridWidth = 4;
    double fieldX = delta * (gridWidth - 1); // 150 m
    double fieldY = delta * ((g_numDrones + gridWidth - 1) / gridWidth - 1); // 100 m

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX",
                                  DoubleValue(0.0),
                                  "MinY",
                                  DoubleValue(0.0),
                                  "DeltaX",
                                  DoubleValue(delta),
                                  "DeltaY",
                                  DoubleValue(delta),
                                  "GridWidth",
                                  UintegerValue(gridWidth),
                                  "LayoutType",
                                  StringValue("RowFirst"));

    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                              "Mode",
                              StringValue("Time"),
                              "Time",
                              StringValue("2s"),
                              "Speed",
                              StringValue("ns3::UniformRandomVariable[Min=1.0|Max=3.0]"),
                              "Bounds",
                              RectangleValue(Rectangle(0.0, fieldX, 0.0, fieldY)));
    mobility.Install(drones);
    mobility.AssignStreams(drones, 100);

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("ErpOfdmRate6Mbps"),
                                 "ControlMode",
                                 StringValue("ErpOfdmRate6Mbps"));

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    channel.AddPropagationLoss("ns3::RangePropagationLossModel",
                               "MaxRange",
                               DoubleValue(g_wifiRangeM));
    YansWifiPhyHelper phy;
    phy.Set("TxPowerStart", DoubleValue(22.0));
    phy.Set("TxPowerEnd", DoubleValue(22.0));
    phy.SetChannel(channel.Create());

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(phy, mac, drones);
    wifi.AssignStreams(devices, 200);

    InternetStackHelper internet;
    Ipv4StaticRoutingHelper staticRoutingHelper;
    internet.SetRoutingHelper(staticRoutingHelper);
    internet.Install(drones);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    BasicEnergySourceHelper energySourceHelper;
    EnergySourceContainer energySources;
    for (uint32_t i = 0; i < g_numDrones; ++i)
    {
        double initialEnergy = InitialEnergyForNode(i);
        g_initialEnergyPerNode[i] = initialEnergy;
        g_remainingEnergy[i] = initialEnergy;
        energySourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(initialEnergy));
        NodeContainer oneDrone;
        oneDrone.Add(drones.Get(i));
        EnergySourceContainer oneSource = energySourceHelper.Install(oneDrone);
        energySources.Add(oneSource.Get(0));
        g_sources.push_back(DynamicCast<BasicEnergySource>(oneSource.Get(0)));
    }

    WifiRadioEnergyModelHelper radioEnergyHelper;
    // Idle current is reduced relative to a typical Wi-Fi datasheet so that
    // forwarding energy is visible. At 0.273 A, idle alone uses ~164 J in
    // 200 s and swamps any routing-induced difference.
    radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.380));
    radioEnergyHelper.Set("RxCurrentA", DoubleValue(0.313));
    radioEnergyHelper.Set("IdleCurrentA", DoubleValue(0.120));
    radioEnergyHelper.Install(devices, energySources);

    for (uint32_t i = 0; i < g_numDrones; ++i)
    {
        g_sources[i]->TraceConnectWithoutContext("RemainingEnergy",
                                                 MakeBoundCallback(&RemainingEnergyCallback, i));
    }
    Simulator::Schedule(Seconds(0.0), &LogEnergyTrace);

    EnergyAwareGlobalRouting router(drones, interfaces);
    router.Start();

    // Deterministic far-peer CBR: UAV i sends to UAV (i + n/2) mod n so
    // most pairs need at least one relay on the grid.
    uint16_t basePort = 9000;
    ApplicationContainer sinkApps;
    ApplicationContainer sourceApps;
    for (uint32_t i = 0; i < g_numDrones; ++i)
    {
        uint16_t port = basePort + i;
        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), port));
        sinkApps.Add(sink.Install(drones.Get(i)));
    }
    for (uint32_t i = 0; i < g_numDrones; ++i)
    {
        uint32_t dest = (i + g_numDrones / 2) % g_numDrones;
        uint16_t destPort = basePort + dest;
        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(interfaces.GetAddress(dest), destPort));
        onoff.SetAttribute("DataRate", StringValue("128kbps"));
        onoff.SetAttribute("PacketSize", UintegerValue(256));
        onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer app = onoff.Install(drones.Get(i));
        app.Start(Seconds(1.0 + 0.05 * i));
        app.Stop(Seconds(g_simTime - 1.0));
        sourceApps.Add(app);
    }
    sinkApps.Start(Seconds(0.5));
    sinkApps.Stop(Seconds(g_simTime));

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    Simulator::Stop(Seconds(g_simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
    monitor->SerializeToXmlFile(WithPrefix(g_flowMonFile), true, true);
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    WriteMetrics(monitor, classifier);

    Simulator::Destroy();
    g_energyTraceOut.close();
    g_routingEventsOut.close();
    return 0;
}
