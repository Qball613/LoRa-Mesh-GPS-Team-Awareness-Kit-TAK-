# LoRa Mesh Routing Pseudocode

This document outlines the pseudocode for implementing a robust LoRa mesh routing system for the GPS Team Awareness Kit. The system combines multiple routing protocols to handle diverse network conditions including high mobility, varying link quality, energy constraints, and network partitions.

## Architecture Overview

The routing system implements a **hybrid approach** combining:

1. **Reactive Routing (AODV)**: On-demand route discovery when needed
2. **Proactive Routing (Link State)**: Maintains network topology awareness  
3. **Geographic Routing**: Uses GPS coordinates for position-based forwarding
4. **Multi-Path Routing**: Maintains alternate routes for reliability and load balancing

### Design Philosophy

- **Adaptive**: Selects optimal routing strategy based on current network conditions
- **Fault-Tolerant**: Multiple fallback mechanisms for route failures
- **Energy-Aware**: Considers battery levels in routing decisions
- **QoS-Enabled**: Different service levels for various message types
- **Scalable**: Efficient for networks from 10 to 100+ nodes

## Core Data Structures

The following data structures form the foundation of the routing system:

```pseudocode
// Node represents a single device in the mesh network
// Stores both static identity and dynamic state information
Structure Node:
    node_id: String (unique identifier)          // Persistent device identifier
    gps_position: GPSCoordinate                  // Current GPS location
    last_seen: Timestamp                         // Last communication time
    rssi: Integer (signal strength)              // Received signal strength indicator
    hop_count: Integer (distance from source)    // Network distance in hops
    battery_level: Integer (0-100)               // Remaining battery percentage
    is_direct_neighbor: Boolean                  // True if within radio range
    sequence_number: Integer (freshness indicator) // Monotonic counter for route freshness
    link_quality: Float (0.0 to 1.0)            // Calculated link reliability metric
    mobility_factor: Float (speed-based metric)  // Movement speed indicator

// RouteEntry represents a single route to a destination
// Contains all information needed for routing decisions and maintenance
Structure RouteEntry:
    destination_id: String                       // Target node identifier
    next_hop_id: String                         // Next node in the path
    hop_count: Integer                          // Total hops to destination
    destination_sequence_number: Integer         // Freshness of route information
    route_creation_time: Timestamp              // When route was established
    route_expiry_time: Timestamp                // When route becomes invalid
    precursor_list: Set<String>                 // Nodes using this route (for error propagation)
    route_flags: Integer                        // Status flags (valid, invalid, repairable, etc.)
    route_quality_metrics: RouteQualityMetrics // Detailed performance measurements
    backup_routes: List<AlternateRoute>         // Alternative paths to same destination

// RouteQualityMetrics tracks route performance over time
// Used for intelligent route selection and switching decisions
Structure RouteQualityMetrics:
    average_rssi: Float                         // Mean signal strength
    packet_loss_rate: Float                     // Percentage of lost packets
    latency_estimate: Float                     // Expected delivery time
    energy_cost: Float                          // Battery consumption estimate
    stability_factor: Float                     // Route reliability metric

// AlternateRoute represents a backup path to a destination
// Maintained for rapid failover and load balancing
Structure AlternateRoute:
    next_hop_id: String                         // Next hop for this alternate path
    hop_count: Integer                          // Path length in hops
    quality_score: Float                        // Overall path quality rating
    last_used: Timestamp                        // Last time this path was used

// RoutingTable is the core routing data structure
// Maintains all routes and routing state for the local node
Structure RoutingTable:
    routes: Map<String, RouteEntry>             // destination_id -> route information
    sequence_number: Integer                    // Our own sequence number (incremented)
    rreq_cache: Map<String, RREQCacheEntry>    // Prevents RREQ loops and duplicates

// RREQCacheEntry prevents processing duplicate route requests
// Essential for loop prevention in flooding-based route discovery
Structure RREQCacheEntry:
    originator_id: String                       // Node that initiated the RREQ
    rreq_id: Integer                           // Unique request identifier
    expiry_time: Timestamp                     // When to remove from cache

// Message is the universal container for all network communications
// Extensible design supports multiple message types and protocols
Structure Message:
    message_id: String (unique)                 // Globally unique message identifier
    source_id: String                          // Original sender
    destination_id: String                     // Final recipient (or "BROADCAST")
    message_type: Enum (GPS_UPDATE, TEXT, CONTROL, ACK, RREQ, RREP, RERR, LSA)
    payload: Bytes                             // Message content (protocol-specific)
    timestamp: Timestamp                       // Creation time
    ttl: Integer (time to live)                // Hop limit for forwarding
    hop_count: Integer                         // Hops from originator
    sequence_number: Integer                   // Message freshness indicator
    signature: Bytes (cryptographic signature) // Authentication and integrity

// === AODV Protocol Messages ===
// These structures define the AODV route discovery protocol messages

// RouteRequest initiates route discovery to unknown destinations
// Flooded through network until destination or fresh route found
Structure RouteRequest:
    rreq_id: Integer                           // Unique request ID from originator
    originator_id: String                      // Node seeking the route
    originator_sequence_number: Integer        // Freshness of originator's info
    destination_id: String                     // Target node
    destination_sequence_number: Integer       // Last known dest sequence (0 if unknown)
    hop_count: Integer                         // Hops from originator
    broadcast_id: Integer                      // Prevents broadcast loops

// RouteReply sent by destination or intermediate node with fresh route
// Travels back to originator establishing forward route
Structure RouteReply:
    destination_id: String                     // Replying node (destination)
    destination_sequence_number: Integer       // Current sequence number
    originator_id: String                      // Original requester
    hop_count: Integer                         // Hops from destination
    lifetime: Integer                          // Route validity period

// RouteError notifies about broken routes
// Propagated to all nodes using the broken route
Structure RouteError:
    unreachable_destinations: List<UnreachableDestination> // List of broken routes

Structure UnreachableDestination:
    destination_id: String                     // Unreachable node
    destination_sequence_number: Integer       // Incremented sequence number

// === Link State Protocol Messages ===
// For proactive topology maintenance and shortest path computation

// LinkStateAdvertisement contains neighbor connectivity information
// Periodically flooded to maintain network topology awareness
Structure LinkStateAdvertisement:
    originator_id: String                      // Advertising node
    sequence_number: Integer                   // LSA freshness indicator
    neighbors: List<NeighborInfo>              // Current neighbor list
    gps_position: GPSCoordinate               // Originator's location

// NeighborInfo describes a single neighbor link
// Used in LSAs to describe network topology
Structure NeighborInfo:
    neighbor_id: String                        // Neighbor node identifier
    link_quality: Float                        // Link reliability (0.0 to 1.0)
    rssi: Integer                             // Signal strength to neighbor

Structure MessageCache:
    recent_messages: Set<String>  // message IDs seen recently
    forwarded_messages: Set<String>  // messages we've forwarded
    max_cache_size: Integer = 1000
    cache_timeout: Duration = 300 seconds
```

## AODV Route Discovery Protocol

The Ad Hoc On-Demand Distance Vector (AODV) protocol forms the reactive routing backbone of our system. It discovers routes only when needed, reducing overhead while maintaining fresh routing information.

### Route Discovery Process Overview

1. **Route Request (RREQ)**: When a node needs a route to an unknown destination, it broadcasts an RREQ
2. **RREQ Propagation**: Intermediate nodes forward the RREQ while setting up reverse routes
3. **Route Reply (RREP)**: The destination (or a node with a fresh route) unicasts an RREP back
4. **Route Establishment**: The RREP establishes forward routes as it travels back to the originator

### Key Features

- **Loop Prevention**: Uses sequence numbers and broadcast IDs to prevent routing loops
- **Fresh Routes**: Sequence numbers ensure routes use the most recent topology information  
- **Efficient Discovery**: Stops flooding once destination is reached or fresh route found
- **Reverse Route Setup**: Establishes bidirectional communication during discovery

```pseudocode
/**
 * Initiates route discovery for a destination node
 * Called when no valid route exists and a message needs to be sent
 * 
 * @param destination_id: Target node identifier
 * @return: Route to destination if immediately available, null if discovery started
 */
Method initiate_route_discovery(destination_id: String):
    // First check if we already have a valid (non-expired) route
    if has_valid_route(destination_id):
        return get_route(destination_id)
    
    // Prevent multiple simultaneous discoveries to same destination
    // This reduces network overhead and prevents discovery storms
    if is_rreq_in_progress(destination_id):
        return null  // Wait for existing discovery to complete
    
    // Create and broadcast the route request
    rreq = create_route_request(destination_id)
    broadcast_rreq(rreq)
    
    // Set timeout to handle discovery failures
    // If no RREP received within timeout, consider destination unreachable
    start_route_discovery_timeout(destination_id, RREQ_TIMEOUT)

/**
 * Creates a new Route Request message
 * Incorporates AODV sequence number semantics for freshness and loop prevention
 *
 * @param destination_id: Target node for route discovery
 * @return: Configured RouteRequest ready for broadcast
 */
Method create_route_request(destination_id: String) -> RouteRequest:
    // Increment our sequence number to ensure freshness
    // This guarantees that newer information supersedes older data
    routing_table.sequence_number += 1
    
    rreq = new RouteRequest()
    rreq.rreq_id = generate_rreq_id()                    // Unique per originator
    rreq.originator_id = own_node_id                     // We are the originator
    rreq.originator_sequence_number = routing_table.sequence_number
    rreq.destination_id = destination_id
    rreq.hop_count = 0                                   // Start at 0 hops
    rreq.broadcast_id = generate_broadcast_id()          // Prevent broadcast loops
    
    // Set destination sequence number if we have previous route info
    // This helps intermediate nodes determine if their cached routes are fresh enough
    if routing_table.routes.contains(destination_id):
        route = routing_table.routes[destination_id]
        rreq.destination_sequence_number = route.destination_sequence_number
    else:
        rreq.destination_sequence_number = 0  // Unknown destination
    
    // Cache our own RREQ to prevent processing if it comes back to us
    // This is essential for loop prevention in the flooding process
    cache_entry = new RREQCacheEntry()
    cache_entry.originator_id = own_node_id
    cache_entry.rreq_id = rreq.rreq_id
    cache_entry.expiry_time = current_time() + RREQ_CACHE_TIMEOUT
    routing_table.rreq_cache[get_rreq_key(own_node_id, rreq.rreq_id)] = cache_entry
    
    return rreq

/**
 * Processes incoming Route Request messages
 * Implements AODV RREQ handling with duplicate detection and reverse route setup
 *
 * @param rreq: The received Route Request
 * @param sender_id: Node that forwarded this RREQ to us
 */
Method process_route_request(rreq: RouteRequest, sender_id: String):
    // Check for RREQ loops using our cache
    // If we've seen this RREQ before, silently drop it
    rreq_key = get_rreq_key(rreq.originator_id, rreq.rreq_id)
    if routing_table.rreq_cache.contains(rreq_key):
        return  // Already processed this RREQ - prevent loops
    
    // Cache this RREQ to prevent future processing
    cache_entry = new RREQCacheEntry()
    cache_entry.originator_id = rreq.originator_id
    cache_entry.rreq_id = rreq.rreq_id
    cache_entry.expiry_time = current_time() + RREQ_CACHE_TIMEOUT
    routing_table.rreq_cache[rreq_key] = cache_entry
    
    // Establish reverse route to the originator
    // This enables the RREP to travel back and provides bidirectional communication
    create_reverse_route(rreq, sender_id)
    
    // Check if we are the destination node
    if rreq.destination_id == own_node_id:
        // We are the destination - send RREP directly back
        send_route_reply(rreq, sender_id)
        return
    
    // Check if we have a fresh route to the destination
    // An intermediate node can reply if it has fresher information than requested
    if has_fresh_route_to_destination(rreq):
        // Send RREP on behalf of the destination (intermediate node reply)
        send_route_reply_from_intermediate(rreq, sender_id)
        return
    
    // Forward RREQ if TTL allows and we're not the destination
    // This continues the flooding process toward the destination
    if rreq.hop_count < MAX_RREQ_HOPS:
        forward_route_request(rreq)

/**
 * Establishes reverse route to RREQ originator
 * Critical for enabling RREP to travel back and for bidirectional communication
 *
 * @param rreq: The Route Request being processed
 * @param sender_id: Node that sent us this RREQ (becomes next hop back to originator)
 */
Method create_reverse_route(rreq: RouteRequest, sender_id: String):
    route_entry = new RouteEntry()
    route_entry.destination_id = rreq.originator_id
    route_entry.next_hop_id = sender_id                  // Route back through sender
    route_entry.hop_count = rreq.hop_count + 1          // One more hop than RREQ
    route_entry.destination_sequence_number = rreq.originator_sequence_number
    route_entry.route_creation_time = current_time()
    route_entry.route_expiry_time = current_time() + REVERSE_ROUTE_LIFETIME
    route_entry.route_flags = VALID | REVERSE_ROUTE
    
    // Add sender to precursor list for route maintenance
    // If this reverse route breaks, sender should be notified
    route_entry.precursor_list.add(sender_id)
    
    routing_table.routes[rreq.originator_id] = route_entry

/**
 * Generates Route Reply when we are the destination
 * Creates RREP with our current sequence number and sends back to originator
 *
 * @param rreq: The Route Request we're replying to
 * @param sender_id: Node to send the RREP to (next hop back to originator)
 */
Method send_route_reply(rreq: RouteRequest, sender_id: String):
    // Increment our sequence number for freshness
    routing_table.sequence_number += 1
    
    rrep = new RouteReply()
    rrep.destination_id = own_node_id                    // We are the destination
    rrep.destination_sequence_number = routing_table.sequence_number
    rrep.originator_id = rreq.originator_id
    rrep.hop_count = 0                                   // 0 hops from destination
    rrep.lifetime = ACTIVE_ROUTE_LIFETIME               // How long route stays valid
    
    // Send RREP back along the reverse route
    send_message_to_node(create_rrep_message(rrep), sender_id)

/**
 * Processes incoming Route Reply messages
 * Establishes forward routes and forwards RREP toward originator
 *
 * @param rrep: The Route Reply message
 * @param sender_id: Node that sent us this RREP
 */
Method process_route_reply(rrep: RouteReply, sender_id: String):
    // Verify we have a reverse route to the originator
    // We should only process RREPs for discoveries we participated in
    if not routing_table.routes.contains(rrep.originator_id):
        return  // No reverse route - drop RREP
    
    // Check if this route information is better than what we have
    // Only update if this route is fresher or shorter
    if should_update_route(rrep, sender_id):
        update_route_from_rrep(rrep, sender_id)
    
    // Forward RREP toward originator if we're not the originator
    if rrep.originator_id != own_node_id:
        forward_route_reply(rrep)
    else:
        // We're the originator - route discovery complete!
        notify_route_established(rrep.destination_id)

/**
 * Determines if route information in RREP is better than existing route
 * Uses AODV route selection criteria: sequence number freshness, then hop count
 *
 * @param rrep: Route Reply with new route information
 * @param sender_id: Next hop for the new route
 * @return: True if new route should replace existing route
 */
Method should_update_route(rrep: RouteReply, sender_id: String) -> Boolean:
    destination_id = rrep.destination_id
    
    if not routing_table.routes.contains(destination_id):
        return true  // No existing route - accept this one
    
    existing_route = routing_table.routes[destination_id]
    
    // AODV Rule 1: Fresher sequence number always wins
    if rrep.destination_sequence_number > existing_route.destination_sequence_number:
        return true  // Fresher route information
    
    // AODV Rule 2: Same freshness - prefer shorter path
    if rrep.destination_sequence_number == existing_route.destination_sequence_number:
        new_hop_count = rrep.hop_count + 1
        if new_hop_count < existing_route.hop_count:
            return true  // Shorter route with same freshness
    
    return false  // Existing route is better or equal

/**
 * Updates routing table with information from Route Reply
 * Creates new route entry with comprehensive metrics and quality assessment
 *
 * @param rrep: Route Reply containing route information
 * @param sender_id: Next hop node for this route
 */
Method update_route_from_rrep(rrep: RouteReply, sender_id: String):
    route_entry = new RouteEntry()
    route_entry.destination_id = rrep.destination_id
    route_entry.next_hop_id = sender_id                  // Route through sender
    route_entry.hop_count = rrep.hop_count + 1          // Add our hop
    route_entry.destination_sequence_number = rrep.destination_sequence_number
    route_entry.route_creation_time = current_time()
    route_entry.route_expiry_time = current_time() + rrep.lifetime
    route_entry.route_flags = VALID | FORWARD_ROUTE
    
    // Calculate initial route quality metrics
    route_entry.route_quality_metrics = calculate_comprehensive_route_quality(sender_id, rrep)
    
    // Store as primary route
    routing_table.routes[rrep.destination_id] = route_entry
    
    // Consider adding to alternate routes list if not primary
    maintain_alternate_routes(rrep.destination_id, route_entry)
```

## Multi-Path Route Discovery

Multi-path routing enhances reliability and enables load balancing by maintaining multiple routes to each destination. This is particularly valuable in mobile networks where routes frequently change.

### Benefits of Multi-Path Routing

- **Reliability**: If primary route fails, alternate routes provide immediate fallback
- **Load Balancing**: Distribute traffic across multiple paths to prevent bottlenecks  
- **Reduced Latency**: Route discovery overhead reduced when alternates available
- **Network Utilization**: Better use of available network capacity

```pseudocode
/**
 * Discovers multiple paths to a destination simultaneously
 * Uses staggered RREQs with different optimization criteria for path diversity
 *
 * @param destination_id: Target node
 * @param max_paths: Maximum number of paths to discover
 */
Method discover_multiple_paths(destination_id: String, max_paths: Integer):
    // Launch multiple route discoveries with different optimization preferences
    // This encourages discovery of diverse paths through different network regions
    for path_index = 0 to max_paths - 1:
        rreq = create_multipath_rreq(destination_id, path_index)
        
        // Stagger broadcasts to reduce collisions and interference
        // Each RREQ gets a slightly different view of network state
        delay = path_index * MULTIPATH_RREQ_DELAY
        schedule_broadcast(rreq, delay)

/**
 * Creates Route Request optimized for specific path characteristics
 * Different optimization preferences encourage diverse path discovery
 *
 * @param destination_id: Target destination
 * @param path_index: Index determining optimization preference
 * @return: Configured RouteRequest for multipath discovery
 */
Method create_multipath_rreq(destination_id: String, path_index: Integer) -> RouteRequest:
    rreq = create_route_request(destination_id)
    
    // Modify RREQ ID to make each path discovery unique
    rreq.rreq_id += path_index  // Ensures different cache keys
    
    // Set different optimization preferences to encourage path diversity
    // Each preference guides intermediate nodes in path selection decisions
    switch path_index:
        case 0:
            rreq.preferred_metric = SHORTEST_HOP     // Minimize hop count
        case 1:
            rreq.preferred_metric = HIGHEST_RSSI     // Best signal quality
        case 2:
            rreq.preferred_metric = LOWEST_ENERGY    // Battery conservation
        case 3:
            rreq.preferred_metric = MOST_STABLE      // Least mobile nodes
    
    return rreq

/**
 * Maintains alternate routes alongside primary route
 * Intelligently promotes/demotes routes based on quality comparisons
 *
 * @param destination_id: Destination for route maintenance
 * @param new_route: Newly discovered route to evaluate
 */
Method maintain_alternate_routes(destination_id: String, new_route: RouteEntry):
    if not routing_table.routes.contains(destination_id):
        // No existing route - this becomes the primary
        routing_table.routes[destination_id] = new_route
        return
    
    primary_route = routing_table.routes[destination_id]
    
    // Compare new route against current primary
    if is_better_route(new_route, primary_route):
        // New route is better - promote to primary, demote old primary
        add_alternate_route(destination_id, primary_route)
        routing_table.routes[destination_id] = new_route
    else:
        // Add as alternate route for failover and load balancing
        add_alternate_route(destination_id, new_route)

/**
 * Adds route to alternate routes list with quality-based ordering
 * Maintains a bounded list of best alternate routes
 *
 * @param destination_id: Destination identifier
 * @param route: Route to add to alternates list
 */
Method add_alternate_route(destination_id: String, route: RouteEntry):
    primary_route = routing_table.routes[destination_id]
    
    // Convert route entry to alternate route format
    alternate = new AlternateRoute()
    alternate.next_hop_id = route.next_hop_id
    alternate.hop_count = route.hop_count
    alternate.quality_score = calculate_overall_quality_score(route)
    alternate.last_used = current_time()
    
    // Add to backup routes list
    primary_route.backup_routes.add(alternate)
    
    // Maintain bounded list of best alternates to limit memory usage
    if primary_route.backup_routes.size() > MAX_ALTERNATE_ROUTES:
        primary_route.backup_routes.sort_by_quality_desc()         // Best first
        primary_route.backup_routes = primary_route.backup_routes.take(MAX_ALTERNATE_ROUTES)
```
```

## Geographic Routing Protocol

Geographic routing leverages GPS coordinates to make forwarding decisions without maintaining detailed topology information. This approach is particularly effective in mobile networks where topology-based routes become stale quickly.

### Geographic Routing Strategies

1. **Greedy Forwarding**: Forward to neighbor closest to destination
2. **Perimeter Routing**: When greedy fails, route around obstacles using right-hand rule
3. **Predictive Routing**: Account for node mobility by predicting future positions

### Advantages of Geographic Routing

- **Scalability**: Routing decisions use only local neighbor information
- **Mobility Resilience**: Adapts quickly to topology changes without route discovery
- **Reduced Overhead**: No need to maintain routes or flood control messages
- **Natural Load Balancing**: Traffic naturally spreads across geographic regions

```pseudocode
/**
 * Routes message using GPS coordinates of destination
 * Selects appropriate geographic routing strategy based on network conditions
 *
 * @param destination_id: Target node identifier
 * @param message: Message to route
 * @return: True if message successfully forwarded
 */
Method geographic_route_to_destination(destination_id: String, message: Message) -> Boolean:
    // Obtain GPS coordinates of destination node
    destination_position = get_node_position(destination_id)
    
    if destination_position == null:
        // No geographic information available - fall back to topology routing
        log("No GPS info for " + destination_id + ", using topology routing")
        return topology_route_to_destination(destination_id, message)
    
    current_position = gps_module.get_position()
    
    // Select optimal geographic routing strategy based on local conditions
    routing_mode = select_geographic_routing_mode(destination_position, current_position)
    
    switch routing_mode:
        case GREEDY_FORWARDING:
            return greedy_geographic_forward(destination_position, message)
        case PERIMETER_ROUTING:
            return perimeter_routing_forward(destination_position, message)
        case PREDICTIVE_ROUTING:
            return predictive_geographic_forward(destination_id, message)

/**
 * Implements greedy geographic forwarding with collision avoidance
 * Handles multiple nodes receiving the same message to prevent duplicate forwarding
 *
 * @param destination_position: GPS coordinates of destination
 * @param message: Message to forward
 * @return: True if forwarding successful
 */
Method greedy_geographic_forward(destination_position: GPSCoordinate, message: Message) -> Boolean:
    current_position = gps_module.get_position()
    current_distance = calculate_distance(current_position, destination_position)
    
    // Check if this message has already been processed (duplicate detection)
    if message_cache.contains(message.message_id):
        log("Message " + message.message_id + " already processed, not forwarding")
        return false  // Already seen this message
    
    // Add to message cache to prevent future processing
    message_cache.add(message.message_id)
    
    best_neighbor = null
    best_distance = current_distance  // Must be closer than us
    
    // Evaluate all direct neighbors for geographic progress
    for each neighbor in get_direct_neighbors():
        neighbor_position = neighbor.gps_position
        
        if neighbor_position != null:
            distance_to_dest = calculate_distance(neighbor_position, destination_position)
            
            // Greedy criterion: neighbor must be closer to destination than we are
            if distance_to_dest < best_distance:
                // Verify link quality to ensure reliable delivery
                link_quality = calculate_link_quality(neighbor)
                
                if link_quality > MIN_GEOGRAPHIC_LINK_QUALITY:
                    best_neighbor = neighbor
                    best_distance = distance_to_dest
    
    if best_neighbor != null:
        // Calculate forwarding suitability score for collision avoidance
        forwarding_score = calculate_geographic_forwarding_score(current_position, 
                                                               destination_position, 
                                                               best_neighbor)
        
        // Implement probabilistic forwarding delay based on suitability
        forwarding_delay = calculate_geographic_forwarding_delay(forwarding_score, message)
        
        // Schedule forwarding with delay to allow better forwarders to go first
        if forwarding_delay > 0:
            log("Scheduling geographic forward after " + forwarding_delay + "ms delay")
            schedule_delayed_forward(message, best_neighbor.node_id, forwarding_delay)
            return true
        else:
            // We're the best forwarder - send immediately
            log("Immediate geographic forward to " + best_neighbor.node_id + 
                " (distance reduced by " + (current_distance - best_distance) + "m)")
            return forward_message_to_neighbor(message, best_neighbor.node_id)
    else:
        // Greedy forwarding failed (local minimum) - switch to perimeter routing
        log("Greedy forwarding failed, switching to perimeter routing")
        return perimeter_routing_forward(destination_position, message)

/**
 * Calculates geographic forwarding score to determine forwarding priority
 * Higher scores indicate better forwarding candidates (lower delay)
 *
 * @param current_position: Our GPS position
 * @param destination_position: Target destination position
 * @param best_neighbor: Selected neighbor for forwarding
 * @return: Forwarding score (0.0 to 1.0, higher is better)
 */
Method calculate_geographic_forwarding_score(current_position: GPSCoordinate,
                                           destination_position: GPSCoordinate,
                                           best_neighbor: Node) -> Float:
    // Factor 1: Geographic progress toward destination
    current_distance = calculate_distance(current_position, destination_position)
    neighbor_distance = calculate_distance(best_neighbor.gps_position, destination_position)
    progress_factor = (current_distance - neighbor_distance) / current_distance
    
    // Factor 2: Link quality to selected neighbor
    link_quality_factor = calculate_link_quality(best_neighbor)
    
    // Factor 3: Battery level (higher battery = better forwarder)
    battery_factor = best_neighbor.battery_level / 100.0
    
    // Factor 4: Node stability (less mobile nodes are better forwarders)
    stability_factor = calculate_node_stability_factor(best_neighbor)
    
    // Weighted combination of factors
    score = progress_factor * 0.4 +        // Geographic progress is most important
           link_quality_factor * 0.3 +     // Link quality ensures delivery
           battery_factor * 0.2 +          // Battery conservation
           stability_factor * 0.1          // Stability helps reliability
    
    return min(max(score, 0.0), 1.0)      // Clamp to [0.0, 1.0]

/**
 * Calculates forwarding delay based on suitability score
 * Better forwarders get shorter delays, preventing suboptimal forwarding
 *
 * @param forwarding_score: Score from 0.0 to 1.0 (higher is better)
 * @param message: Message being forwarded (priority affects timing)
 * @return: Delay in milliseconds before forwarding
 */
Method calculate_geographic_forwarding_delay(forwarding_score: Float, message: Message) -> Integer:
    // Base delay range based on message priority
    base_delay_range = get_base_delay_range_for_priority(message.message_type)
    
    // Invert score so better forwarders get shorter delays
    delay_factor = 1.0 - forwarding_score
    
    // Calculate delay with some randomization to prevent synchronization
    base_delay = delay_factor * base_delay_range.max_delay
    random_jitter = random_float(0, base_delay_range.jitter_range)
    
    total_delay = base_delay + random_jitter
    
    // Critical messages get reduced delays
    if get_message_priority(message) == CRITICAL:
        total_delay *= 0.5  // Halve delay for critical messages
    
    return Integer(total_delay)

/**
 * Scheduled forwarding with cancellation if already forwarded by better node
 * Prevents duplicate transmissions while ensuring message delivery
 *
 * @param message: Message to forward
 * @param next_hop_id: Selected next hop
 * @param delay: Delay before forwarding
 */
Method schedule_delayed_forward(message: Message, next_hop_id: String, delay: Integer):
    // Schedule the forwarding
    timer_id = start_timer(delay, lambda: {
        // Check if message was already forwarded by someone else
        if is_message_already_forwarded(message.message_id):
            log("Message " + message.message_id + " already forwarded by another node")
            return  // Cancel our forwarding
        
        // Check if we still have the route and neighbor is reachable
        if is_neighbor_reachable(next_hop_id):
            // Mark as forwarded before sending to prevent further processing
            mark_message_as_forwarded(message.message_id)
            forward_message_to_neighbor(message, next_hop_id)
            log("Delayed geographic forward completed to " + next_hop_id)
        else:
            log("Next hop " + next_hop_id + " no longer reachable, canceling forward")
        }
    })
    
    // Store timer ID for potential cancellation
    store_forwarding_timer(message.message_id, timer_id)

/**
 * Enhanced duplicate detection that accounts for geographic forwarding
 * Tracks messages that have been forwarded to prevent loops and duplicates
 *
 * @param message_id: Unique message identifier
 * @return: True if message already forwarded by any node
 */
Method is_message_already_forwarded(message_id: String) -> Boolean:
    // Check our local forwarding cache
    if message_cache.forwarded_messages.contains(message_id):
        return true
    
    // Listen for recent transmissions of this message ID
    // If we hear the same message being forwarded by others, we can cancel ours
    recent_transmission = detect_recent_transmission(message_id)
    if recent_transmission != null:
        // Another node forwarded this message recently
        log("Detected forwarding of " + message_id + " by " + recent_transmission.sender_id)
        return true
    
    return false

/**
 * Detects recent transmissions of the same message by other nodes
 * Uses radio monitoring to identify duplicate forwarding attempts
 *
 * @param message_id: Message ID to check for
 * @return: Recent transmission info if detected, null otherwise
 */
Method detect_recent_transmission(message_id: String) -> RecentTransmission:
    // Check recent radio activity for this message ID
    // This requires monitoring radio traffic for message headers
    
    monitoring_window = 2000  // 2 second window
    recent_transmissions = get_recent_radio_activity(monitoring_window)
    
    for each transmission in recent_transmissions:
        if transmission.message_id == message_id and 
           transmission.sender_id != own_node_id:
            // Found recent transmission by another node
            return transmission
    
    return null

/**
 * Geographic broadcast forwarding for multiple receivers
 * Coordinates multiple nodes to prevent broadcast storms
 *
 * @param message: Broadcast message
 * @param sender_id: Node that sent us this broadcast
 * @return: True if forwarding decision made
 */
Method geographic_broadcast_forward(message: Message, sender_id: String) -> Boolean:
    // Special handling for broadcast messages in geographic routing
    
    // Calculate our geographic forwarding priority for this broadcast
    broadcast_priority = calculate_broadcast_forwarding_priority(message, sender_id)
    
    // Use priority-based delay to coordinate multiple forwarders
    forwarding_delay = calculate_broadcast_forwarding_delay(broadcast_priority)
    
    log("Broadcast priority: " + broadcast_priority + ", delay: " + forwarding_delay + "ms")
    
    if forwarding_delay > 0:
        // Schedule delayed broadcast with suppression check
        schedule_delayed_broadcast(message, forwarding_delay)
        return true
    else:
        // High priority - broadcast immediately
        return broadcast_message_to_geographic_region(message)

/**
 * Calculates broadcast forwarding priority based on geographic position
 * Nodes closer to network edge or in sparse regions get higher priority
 *
 * @param message: Broadcast message
 * @param sender_id: Node that sent the broadcast
 * @return: Priority score (0.0 to 1.0, higher = higher priority)
 */
Method calculate_broadcast_forwarding_priority(message: Message, sender_id: String) -> Float:
    // Factor 1: Distance from sender (further nodes have higher priority)
    sender_position = get_node_position(sender_id)
    current_position = gps_module.get_position()
    
    if sender_position != null:
        distance_from_sender = calculate_distance(current_position, sender_position)
        max_radio_range = get_max_radio_range()
        distance_factor = min(distance_from_sender / max_radio_range, 1.0)
    else:
        distance_factor = 0.5  // Unknown sender position
    
    // Factor 2: Local node density (sparse areas need more forwarding)
    local_density = calculate_local_node_density()
    density_factor = 1.0 - min(local_density / 10.0, 1.0)  // Lower density = higher priority
    
    // Factor 3: Network edge detection (edge nodes forward more)
    edge_factor = calculate_network_edge_factor()
    
    // Factor 4: Battery level (higher battery = higher priority)
    battery_factor = get_battery_level() / 100.0
    
    // Weighted combination
    priority = distance_factor * 0.3 +
              density_factor * 0.3 +
              edge_factor * 0.2 +
              battery_factor * 0.2
    
    return min(max(priority, 0.0), 1.0)

/**
 * Gets base delay range based on message priority
 * Critical messages get faster forwarding, routine messages are slower
 *
 * @param message_type: Type of message being forwarded
 * @return: Delay range configuration
 */
Method get_base_delay_range_for_priority(message_type: MessageType) -> DelayRange:
    switch message_type:
        case CONTROL:
            return new DelayRange(max_delay: 50, jitter_range: 20)    // 0-70ms
        case GPS_UPDATE:
            return new DelayRange(max_delay: 200, jitter_range: 100)  // 0-300ms
        case TEXT:
            return new DelayRange(max_delay: 500, jitter_range: 200)  // 0-700ms
        default:
            return new DelayRange(max_delay: 1000, jitter_range: 500) // 0-1500ms

// Supporting data structures for collision avoidance
Structure DelayRange:
    max_delay: Integer        // Maximum base delay in milliseconds
    jitter_range: Integer     // Additional random jitter range

Structure RecentTransmission:
    message_id: String        // Message identifier
    sender_id: String         // Node that transmitted
    timestamp: Timestamp      // When transmission occurred
    rssi: Integer            // Signal strength received

/**
 * Implements predictive geographic forwarding for mobile networks
 * Predicts future position of destination and routes toward predicted location
 *
 * @param destination_id: Target node (for mobility tracking)
 * @param message: Message to forward
 * @return: True if forwarding successful
 */
Method predictive_geographic_forward(destination_id: String, message: Message) -> Boolean:
    // Get current position and velocity of destination
    destination_position = get_node_position(destination_id)
    destination_velocity = estimate_node_velocity(destination_id)
    
    // Estimate time for message to reach destination
    prediction_time = estimate_delivery_time(destination_id)
    
    // Predict where destination will be when message arrives
    predicted_position = predict_future_position(destination_position, 
                                                destination_velocity, 
                                                prediction_time)
    
    log("Predicting " + destination_id + " will move " + 
        calculate_distance(destination_position, predicted_position) + 
        "m in " + prediction_time + "s")
    
    // Route towards predicted position instead of current position
    return greedy_geographic_forward(predicted_position, message)

/**
 * Estimates node velocity from recent position history
 * Uses multiple position samples for smoothed velocity calculation
 *
 * @param node_id: Node to calculate velocity for
 * @return: Estimated velocity vector (speed and direction)
 */
Method estimate_node_velocity(node_id: String) -> Velocity:
    position_history = get_position_history(node_id, VELOCITY_CALCULATION_WINDOW)
    
    if position_history.size() < 2:
        return new Velocity(0, 0)  // Assume stationary if insufficient data
    
    // Calculate velocity from recent position changes
    recent_positions = position_history.get_last_n(3)  // Last 3 positions
    velocity_samples = []
    
    for i = 1 to recent_positions.size() - 1:
        pos1 = recent_positions[i-1]
        pos2 = recent_positions[i]
        time_diff = pos2.timestamp - pos1.timestamp
        
        if time_diff > 0:
            distance = calculate_distance(pos1.coordinate, pos2.coordinate)
            speed = distance / time_diff                    // m/s
            bearing = calculate_bearing(pos1.coordinate, pos2.coordinate)  // degrees
            
            velocity_samples.add(new Velocity(speed, bearing))
    
    // Average velocities for noise reduction
    return average_velocities(velocity_samples)

/**
 * Implements perimeter routing around obstacles
 * Uses right-hand rule to route around voids in network connectivity
 *
 * @param destination_position: GPS coordinates of destination
 * @param message: Message to forward
 * @return: True if forwarding successful
 */
Method perimeter_routing_forward(destination_position: GPSCoordinate, message: Message) -> Boolean:
    // Perimeter routing follows network boundary using computational geometry
    current_position = gps_module.get_position()
    
    // Find the perimeter edge to follow using right-hand rule
    // This requires identifying the network boundary around the void
    perimeter_edge = find_perimeter_edge(current_position, destination_position)
    
    if perimeter_edge != null:
        // Follow perimeter edge to next hop
        next_hop = get_next_perimeter_hop(perimeter_edge)
        
        if next_hop != null:
            log("Perimeter routing via " + next_hop + " around geographic void")
            forward_message_to_neighbor(message, next_hop)
            return true
    
    log("Perimeter routing failed - no viable path found")
    return false  // Perimeter routing failed - destination may be unreachable

/**
 * Predicts future position based on current position and velocity
 * Simple linear prediction - could be enhanced with acceleration
 *
 * @param current_position: Current GPS coordinates
 * @param velocity: Current velocity vector
 * @param time_horizon: Prediction time in seconds
 * @return: Predicted future position
 */
Method predict_future_position(current_position: GPSCoordinate, 
                              velocity: Velocity, 
                              time_horizon: Float) -> GPSCoordinate:
    // Simple linear prediction: position = current + velocity * time
    distance_moved = velocity.speed * time_horizon
    
    // Calculate new position using bearing and distance
    predicted_lat = current_position.latitude + 
                   (distance_moved * cos(velocity.bearing)) / METERS_PER_DEGREE_LAT
    
    predicted_lon = current_position.longitude + 
                   (distance_moved * sin(velocity.bearing)) / 
                   (METERS_PER_DEGREE_LON * cos(current_position.latitude))
    
    return new GPSCoordinate(predicted_lat, predicted_lon)

/**
 * Estimates message delivery time for predictive routing
 * Considers hop count, link quality, and network congestion
 *
 * @param destination_id: Target node
 * @return: Estimated delivery time in seconds
 */
Method estimate_delivery_time(destination_id: String) -> Float:
    // Base estimate on topology distance if available
    if routing_table.routes.contains(destination_id):
        route = routing_table.routes[destination_id]
        hop_count = route.hop_count
        
        // Estimate per-hop delay based on link quality and congestion
        base_hop_delay = 0.5  // seconds per hop baseline
        quality_factor = route.route_quality_metrics.packet_loss_rate + 1.0
        congestion_factor = calculate_network_congestion_factor()
        
        return hop_count * base_hop_delay * quality_factor * congestion_factor
    else:
        // No topology route - estimate based on geographic distance
        current_position = gps_module.get_position()
        destination_position = get_node_position(destination_id)
        
        if destination_position != null:
            distance = calculate_distance(current_position, destination_position)
            estimated_hops = distance / AVERAGE_RADIO_RANGE
            
            return estimated_hops * 0.5  // 500ms per hop estimate
        else:
            return 10.0  // Default 10 second estimate
    }
```
```

## Advanced Main Routing Algorithm

The main routing algorithm orchestrates multiple routing protocols and makes intelligent decisions about which approach to use for each message. This hybrid approach maximizes network performance across diverse operating conditions.

### Routing Strategy Selection

The system dynamically selects the optimal routing strategy based on:

- **Route Availability**: Whether topology-based routes exist
- **Geographic Information**: Availability of GPS coordinates
- **Network Conditions**: Density, mobility, congestion levels
- **Message Priority**: Critical messages get most reliable routing
- **Energy Constraints**: Battery levels influence routing decisions

### Protocol Integration

1. **Reactive (AODV)**: For reliable, established paths
2. **Proactive (Link State)**: For network-wide topology awareness
3. **Geographic**: For mobile scenarios and when topology routes unavailable
4. **Hybrid**: Combines multiple approaches for optimal performance

```pseudocode
/**
 * Advanced LoRa Mesh Router - Main routing engine class
 * Coordinates multiple routing protocols and provides intelligent routing decisions
 */
Class AdvancedLoRaMeshRouter:
    routing_table: RoutingTable                     // AODV routing information
    neighbor_table: Map<String, Node>               // Direct neighbor information
    message_cache: MessageCache                     // Duplicate prevention
    own_node_id: String                            // This node's identifier
    lora_radio: LoRaRadio                          // Hardware radio interface
    gps_module: GPS                                // GPS positioning
    route_discovery_queue: Queue<RouteDiscoveryRequest>  // Pending discoveries
    link_state_database: Map<String, LinkStateAdvertisement> // Network topology
    
    /**
     * Initialize routing system and start all periodic processes
     * Sets up data structures and launches background tasks
     */
    Method initialize():
        routing_table = new RoutingTable()
        neighbor_table = new Map<String, Node>()
        message_cache = new MessageCache()
        own_node_id = generate_unique_id()
        routing_table.sequence_number = 1            // Start sequence numbering
        start_periodic_tasks()                       // Launch background processes
        initialize_routing_protocols()              // Configure protocol weights
    
    /**
     * Configure routing protocol preferences based on network characteristics
     * Adapts the system for different deployment scenarios
     */
    Method initialize_routing_protocols():
        // Enable multiple routing protocols for hybrid operation
        enable_reactive_routing = true               // AODV on-demand routing
        enable_proactive_routing = true             // Link state topology maintenance
        enable_geographic_routing = true            // GPS-based forwarding
        enable_hybrid_routing = true                // Intelligent protocol combination
        
        // Set initial protocol weights based on network conditions
        // These are dynamically adjusted during operation
        update_routing_protocol_weights()
    
    /**
     * Launch all periodic background tasks
     * Each task runs on its own schedule to maintain network state
     */
    Method start_periodic_tasks():
        // Neighbor discovery and maintenance - frequent for mobility
        start_timer(NEIGHBOR_HELLO_INTERVAL, send_hello_message)
        
        // Link state advertisements - moderate frequency for topology
        start_timer(LSA_INTERVAL, send_link_state_advertisement)
        
        // Route maintenance and cleanup - periodic housekeeping
        start_timer(ROUTE_CLEANUP_INTERVAL, cleanup_routing_table)
        
        // GPS updates - frequent for geographic routing
        start_timer(GPS_UPDATE_INTERVAL, send_gps_update)
        
        // Route discovery processing - continuous background task
        start_background_task(process_route_discovery_queue)
        
        // Network analysis - infrequent but comprehensive
        start_timer(TOPOLOGY_ANALYSIS_INTERVAL, analyze_network_topology)
        
        // Message reception - continuous listening
        start_background_task(listen_for_messages)
    
    /**
     * Main message routing entry point
     * Analyzes message and selects optimal routing strategy
     *
     * @param message: Message to route to its destination
     * @return: True if message successfully forwarded
     */
    Method route_message(message: Message) -> Boolean:
        destination_id = message.destination_id
        
        // Handle broadcast messages (special case)
        if destination_id == "BROADCAST":
            log("Broadcasting message " + message.message_id)
            return broadcast_message(message)
        
        // Check if we are the final destination
        if destination_id == own_node_id:
            log("Message " + message.message_id + " reached destination")
            process_message_for_self(message)
            return true
        
        // Select optimal routing strategy for this message
        routing_strategy = select_optimal_routing_strategy(destination_id, message)
        
        // Route using selected strategy
        switch routing_strategy:
            case TOPOLOGY_BASED:
                log("Using topology-based routing for " + destination_id)
                return topology_route_message(message)
            case GEOGRAPHIC_BASED:
                log("Using geographic routing for " + destination_id)
                return geographic_route_to_destination(destination_id, message)
            case HYBRID_ROUTING:
                log("Using hybrid routing for " + destination_id)
                return hybrid_route_message(message)
            case MULTICAST_ROUTING:
                log("Using multicast routing")
                return multicast_route_message(message)
        
        log("All routing strategies failed for " + destination_id)
        return false

    /**
     * Intelligent routing strategy selection based on multiple factors
     * Uses decision matrix to choose optimal approach for current conditions
     *
     * @param destination_id: Target node
     * @param message: Message being routed (affects priority)
     * @return: Selected routing strategy
     */
    Method select_optimal_routing_strategy(destination_id: String, message: Message) -> RoutingStrategy:
        // Gather decision factors
        factors = new RoutingDecisionFactors()
        factors.has_topology_route = routing_table.routes.contains(destination_id)
        factors.has_geographic_info = has_geographic_info(destination_id)
        factors.network_density = calculate_network_density()
        factors.mobility_level = calculate_mobility_level()
        factors.message_priority = get_message_priority(message)
        factors.energy_constraints = get_energy_constraints()
        
        log("Routing decision factors: topology=" + factors.has_topology_route + 
            ", geo=" + factors.has_geographic_info + ", density=" + factors.network_density)
        
        // Decision matrix - prioritized routing strategy selection
        
        // Rule 1: Critical messages use most reliable available route
        if factors.message_priority == CRITICAL and factors.has_topology_route:
            return TOPOLOGY_BASED  // Proven reliable path
        
        // Rule 2: High mobility favors geographic routing
        if factors.mobility_level > HIGH_MOBILITY and factors.has_geographic_info:
            return GEOGRAPHIC_BASED  // Adapts quickly to movement
        
        // Rule 3: Sparse networks benefit from geographic routing
        if factors.network_density < LOW_DENSITY:
            return GEOGRAPHIC_BASED  // Better for sparse topologies
        
        // Rule 4: Use hybrid when both topology and geographic available
        if factors.has_topology_route and factors.has_geographic_info:
            return HYBRID_ROUTING  // Best of both approaches
        
        // Rule 5: Fallback preferences
        if factors.has_topology_route:
            return TOPOLOGY_BASED
        
        if factors.has_geographic_info:
            return GEOGRAPHIC_BASED
        
        // Last resort: initiate route discovery and use topology routing
        log("No routes available, initiating discovery for " + destination_id)
        initiate_route_discovery(destination_id)
        return TOPOLOGY_BASED

    /**
     * Hybrid routing combines multiple approaches for optimal performance
     * Tries topology first, falls back to geographic, then flooding
     *
     * @param message: Message to route
     * @return: True if successfully forwarded
     */
    Method hybrid_route_message(message: Message) -> Boolean:
        destination_id = message.destination_id
        
        // Strategy 1: Try topology-based routing (usually fastest and most reliable)
        if has_valid_route(destination_id):
            log("Hybrid: trying topology route first")
            topology_success = topology_route_message(message)
            if topology_success:
                return true
        
        // Strategy 2: Fall back to geographic routing if topology fails
        if has_geographic_info(destination_id):
            log("Hybrid: topology failed, trying geographic")
            return geographic_route_to_destination(destination_id, message)
        
        // Strategy 3: Last resort - controlled flooding with TTL limit
        log("Hybrid: all strategies failed, using controlled flood")
        return controlled_flood_message(message)

    /**
     * Topology-based message routing using established routes
     * Handles route validation, alternate path selection, and error recovery
     *
     * @param message: Message to route
     * @return: True if successfully forwarded
     */
    Method topology_route_message(message: Message) -> Boolean:
        destination_id = message.destination_id
        
        // Check if we have any route to destination
        if not routing_table.routes.contains(destination_id):
            log("No route to " + destination_id + ", initiating discovery")
            initiate_route_discovery(destination_id)
            return false
        
        route = routing_table.routes[destination_id]
        
        // Validate route freshness and status
        if is_route_expired(route) or not is_route_valid(route):
            log("Route to " + destination_id + " is stale or invalid")
            
            // Attempt route repair before giving up
            if can_repair_route(route):
                repair_route(destination_id)
            else:
                initiate_route_discovery(destination_id)
            return false
        
        next_hop = route.next_hop_id
        
        // Verify next hop is reachable
        if not is_neighbor_reachable(next_hop):
            log("Next hop " + next_hop + " unreachable, trying alternates")
            
            // Try alternate routes before declaring failure
            if route.backup_routes.size() > 0:
                return try_alternate_routes(message, route)
            else:
                // No alternates - send route error and rediscover
                send_route_error(destination_id, route.destination_sequence_number)
                initiate_route_discovery(destination_id)
                return false
        
        // Route is valid and next hop reachable - forward message
        update_route_usage_statistics(destination_id)
        log("Forwarding via " + next_hop + " to " + destination_id + 
            " (" + route.hop_count + " hops)")
        return forward_message_to_neighbor(message, next_hop)

    /**
     * Attempts to use alternate routes when primary route fails
     * Promotes successful alternates to primary route for future use
     *
     * @param message: Message to route
     * @param primary_route: Failed primary route with alternates to try
     * @return: True if alternate route successful
     */
    Method try_alternate_routes(message: Message, primary_route: RouteEntry) -> Boolean:
        // Try backup routes in quality order (best first)
        for each alternate in primary_route.backup_routes:
            if is_neighbor_reachable(alternate.next_hop_id):
                log("Trying alternate route via " + alternate.next_hop_id)
                
                success = forward_message_to_neighbor(message, alternate.next_hop_id)
                if success:
                    // Success! Consider promoting this alternate to primary
                    if alternate.quality_score > calculate_overall_quality_score(primary_route):
                        log("Promoting alternate to primary route")
                        promote_alternate_to_primary(message.destination_id, alternate)
                    
                    // Update usage statistics
                    alternate.last_used = current_time()
                    return true
        
        log("All alternate routes failed")
        return false  // All alternates failed
```
```

## Legacy LoRaMeshRouter (Simplified)
Class LoRaMeshRouter:
    routing_table: RoutingTable
    message_cache: MessageCache
    own_node_id: String
    lora_radio: LoRaRadio
    gps_module: GPS
    
    Method initialize():
        routing_table = new RoutingTable()
        message_cache = new MessageCache()
        own_node_id = generate_unique_id()
        start_periodic_tasks()
    
    Method start_periodic_tasks():
        // Send beacon every 30 seconds
        start_timer(30_seconds, send_beacon)
        
        // Clean up routing table every 60 seconds
        start_timer(60_seconds, cleanup_routing_table)
        
        // Send GPS update every 10 seconds
        start_timer(10_seconds, send_gps_update)
        
        // Listen for incoming messages continuously
        start_background_task(listen_for_messages)
    
    Method send_beacon():
        beacon = create_beacon_message()
        broadcast_message(beacon)
    
    Method send_gps_update():
        if gps_has_fix():
            gps_msg = create_gps_message(gps_module.get_position())
            broadcast_message(gps_msg)
    
    Method listen_for_messages():
        while true:
            if lora_radio.has_message():
                raw_message = lora_radio.receive()
                process_incoming_message(raw_message)
```

## Message Processing with Multiple Receiver Handling

When a message is broadcast over LoRa, multiple nodes within radio range receive it simultaneously. Without proper coordination, this leads to the "broadcast storm" problem where many nodes forward the same message, causing network congestion and interference.

### Multiple Receiver Challenges

1. **Packet Duplication**: Multiple nodes forwarding the same message
2. **Collision Storms**: Simultaneous transmissions interfere with each other  
3. **Resource Waste**: Redundant transmissions drain battery and congest network
4. **Hidden Terminal Problem**: Nodes may not detect each other's transmissions

### Solutions Implemented

1. **Priority-Based Delays**: Better forwarders get shorter delays
2. **Transmission Suppression**: Cancel forwarding if already done by others
3. **Geographic Suitability Scoring**: Coordinate based on position and link quality
4. **Radio Monitoring**: Detect duplicate transmissions to prevent redundancy

```pseudocode
/**
 * Processes incoming messages with multiple receiver coordination
 * Handles duplicate detection and coordinates forwarding decisions
 *
 * @param raw_message: Raw received message bytes
 */
Method process_incoming_message_with_coordination(raw_message: Bytes):
    try:
        message = deserialize_message(raw_message)
        
        // Verify message signature first
        if not verify_signature(message):
            log("Invalid signature, dropping message " + message.message_id)
            return
        
        // Enhanced duplicate detection for multiple receivers
        duplicate_status = check_message_duplicate_status(message)
        
        switch duplicate_status:
            case NEW_MESSAGE:
                // First time seeing this message - process normally
                process_new_message(message)
            
            case DUPLICATE_RECEIVED:
                // We've seen this before - ignore silently
                log("Duplicate message " + message.message_id + " ignored")
                return
            
            case FORWARDED_BY_OTHERS:
                // Others have forwarded this - cancel our forwarding
                cancel_pending_forwarding(message.message_id)
                return
        
        // Update routing information from source
        update_routing_table_from_message(message)
        
        // Process based on message type with coordination
        switch message.message_type:
            case BEACON:
                process_beacon_with_coordination(message)
            case GPS_UPDATE:
                process_gps_update_with_coordination(message)
            case TEXT:
                process_text_message_with_coordination(message)
            case CONTROL:
                process_control_message_with_coordination(message)
            case RREQ:
                process_route_request_with_coordination(message)
            case RREP:
                process_route_reply(message)  // RREPs don't need coordination
            case RERR:
                process_route_error(message)  // RERRs don't need coordination
        
    catch Exception as e:
        log("Error processing message: " + e.message)

/**
 * Enhanced duplicate detection that considers network-wide forwarding
 * Prevents multiple nodes from forwarding the same message
 *
 * @param message: Message to check
 * @return: Duplicate status indicating how to handle the message
 */
Method check_message_duplicate_status(message: Message) -> DuplicateStatus:
    message_key = message.message_id
    
    // Check our local cache first
    if message_cache.recent_messages.contains(message_key):
        // We've processed this message before
        
        if message_cache.forwarded_messages.contains(message_key):
            return DUPLICATE_RECEIVED  // We forwarded it already
        else:
            return DUPLICATE_RECEIVED  // We received it but didn't forward
    
    // New message to us - check if others have forwarded it recently
    if detect_recent_forwarding_by_others(message_key):
        // Add to cache but mark as forwarded by others
        message_cache.recent_messages.add(message_key)
        return FORWARDED_BY_OTHERS
    
    // Truly new message
    message_cache.recent_messages.add(message_key)
    return NEW_MESSAGE

/**
 * Processes GPS updates with coordination to prevent broadcast storms
 * Multiple nodes receiving GPS broadcasts coordinate their forwarding
 *
 * @param message: GPS update message
 */
Method process_gps_update_with_coordination(message: Message):
    source_id = message.source_id
    gps_data = deserialize_gps_payload(message.payload)
    
    // Update our routing table with position information
    if neighbor_table.contains(source_id):
        neighbor = neighbor_table[source_id]
        neighbor.gps_position = gps_data.position
        neighbor.last_seen = current_time()
        
        // Update mobility factor based on position change
        neighbor.mobility_factor = calculate_mobility_factor(neighbor)
    
    // Notify application layer about position update
    notify_position_update(source_id, gps_data.position)
    
    // Determine if we should forward this GPS update
    if should_forward_gps_update(message, source_id):
        // Use geographic coordination for forwarding
        geographic_broadcast_forward(message, source_id)
    
    log("GPS update from " + source_id + " at " + gps_data.position.toString())

/**
 * Determines if this node should forward a GPS update
 * Considers network density and geographic position
 *
 * @param message: GPS update message
 * @param source_id: Original sender of the GPS update
 * @return: True if this node should participate in forwarding
 */
Method should_forward_gps_update(message: Message, source_id: String) -> Boolean:
    // Don't forward if TTL expired
    if message.ttl <= 1:
        return false
    
    // Don't forward GPS updates in very dense networks
    local_density = calculate_local_node_density()
    if local_density > HIGH_DENSITY_THRESHOLD:
        // In dense networks, only edge nodes forward GPS updates
        if not is_network_edge_node():
            log("Dense network detected, not forwarding GPS update")
            return false
    
    // Don't forward if we're very close to the source
    source_position = get_node_position(source_id)
    if source_position != null:
        current_position = gps_module.get_position()
        distance_to_source = calculate_distance(current_position, source_position)
        
        if distance_to_source < MIN_FORWARDING_DISTANCE:
            log("Too close to source, not forwarding GPS update")
            return false
    
    // Forward if we can reach nodes that the source cannot
    return can_reach_additional_nodes(source_id)

/**
 * Route request processing with coordination to prevent RREQ storms
 * Multiple nodes coordinate RREQ forwarding to prevent network flooding
 *
 * @param message: Route request message
 */
Method process_route_request_with_coordination(message: Message):
    rreq = deserialize_rreq_payload(message.payload)
    sender_id = get_previous_hop(message)
    
    // Standard AODV RREQ processing
    standard_rreq_result = process_route_request(rreq, sender_id)
    
    if standard_rreq_result == SHOULD_FORWARD:
        // Calculate RREQ forwarding priority
        forwarding_priority = calculate_rreq_forwarding_priority(rreq, sender_id)
        
        // Use priority-based delay to coordinate RREQ forwarding
        forwarding_delay = calculate_rreq_forwarding_delay(forwarding_priority)
        
        if forwarding_delay > 0:
            log("Scheduling RREQ forward after " + forwarding_delay + "ms")
            schedule_delayed_rreq_forward(rreq, forwarding_delay)
        else:
            // High priority - forward immediately
            forward_route_request(rreq)

/**
 * Calculates RREQ forwarding priority to coordinate multiple forwarders
 * Nodes closer to destination or with better connectivity get higher priority
 *
 * @param rreq: Route request being processed
 * @param sender_id: Node that sent us this RREQ
 * @return: Priority score (0.0 to 1.0, higher = higher priority)
 */
Method calculate_rreq_forwarding_priority(rreq: RouteRequest, sender_id: String) -> Float:
    // Factor 1: Geographic proximity to destination (if known)
    geographic_factor = 0.5  // Default if no geographic info
    
    dest_position = get_node_position(rreq.destination_id)
    current_position = gps_module.get_position()
    sender_position = get_node_position(sender_id)
    
    if dest_position != null and current_position != null and sender_position != null:
        current_distance = calculate_distance(current_position, dest_position)
        sender_distance = calculate_distance(sender_position, dest_position)
        
        if current_distance < sender_distance:
            // We're closer to destination than sender
            geographic_factor = 0.8
        else:
            geographic_factor = 0.3
    
    // Factor 2: Node connectivity (more neighbors = higher priority)
    neighbor_count = get_direct_neighbor_count()
    connectivity_factor = min(neighbor_count / 8.0, 1.0)  // Normalize to max 8 neighbors
    
    // Factor 3: Battery level (higher battery = higher priority)
    battery_factor = get_battery_level() / 100.0
    
    // Factor 4: Link quality to sender (better link = higher priority)
    link_quality_factor = calculate_link_quality_to_node(sender_id)
    
    // Weighted combination
    priority = geographic_factor * 0.4 +
              connectivity_factor * 0.3 +
              battery_factor * 0.2 +
              link_quality_factor * 0.1
    
    return min(max(priority, 0.0), 1.0)

/**
 * Radio monitoring for duplicate transmission detection
 * Passively listens for message IDs in other transmissions
 *
 * @param message_id: Message ID to monitor for
 * @param monitoring_duration: How long to listen (milliseconds)
 * @return: True if message ID detected in other transmissions
 */
Method monitor_for_duplicate_transmissions(message_id: String, monitoring_duration: Integer) -> Boolean:
    start_time = current_time()
    
    while (current_time() - start_time) < monitoring_duration:
        // Listen for radio activity
        if lora_radio.has_activity():
            intercepted_header = lora_radio.intercept_header()
            
            if intercepted_header.message_id == message_id:
                intercepted_sender = intercepted_header.source_id
                
                if intercepted_sender != own_node_id:
                    log("Detected duplicate transmission of " + message_id + 
                        " by " + intercepted_sender)
                    return true
        
        // Small delay to prevent busy waiting
        delay(10)  // 10ms
    
    return false

// Supporting enumerations and structures
Enum DuplicateStatus:
    NEW_MESSAGE           // First time seeing this message
    DUPLICATE_RECEIVED    // We've processed this message before
    FORWARDED_BY_OTHERS   // Other nodes have already forwarded this

Structure InterceptedHeader:
    message_id: String    // Message identifier
    source_id: String     // Sender of the message
    destination_id: String // Target of the message
    message_type: MessageType // Type of message
    ttl: Integer         // Time to live remaining

```pseudocode
Method send_link_state_advertisement():
    // Create LSA with current neighbor information
    lsa = new LinkStateAdvertisement()
    lsa.originator_id = own_node_id
    lsa.sequence_number = get_next_lsa_sequence_number()
    lsa.neighbors = build_neighbor_list()
    lsa.gps_position = gps_module.get_position()
    
    // Broadcast LSA to all neighbors
    lsa_message = create_lsa_message(lsa)
    broadcast_message(lsa_message)
    
    // Store in our own database
    link_state_database[own_node_id] = lsa

Method build_neighbor_list() -> List<NeighborInfo>:
    neighbors = []
    
    for each neighbor_id in neighbor_table.keys():
        neighbor = neighbor_table[neighbor_id]
        
        if is_neighbor_active(neighbor):
            neighbor_info = new NeighborInfo()
            neighbor_info.neighbor_id = neighbor_id
            neighbor_info.link_quality = calculate_link_quality(neighbor)
            neighbor_info.rssi = neighbor.rssi
            
            neighbors.add(neighbor_info)
    
    return neighbors

Method process_link_state_advertisement(lsa: LinkStateAdvertisement, sender_id: String):
    // Check if this LSA is newer than what we have
    if should_accept_lsa(lsa):
        // Update link state database
        link_state_database[lsa.originator_id] = lsa
        
        // Recompute shortest paths
        recompute_shortest_paths()
        
        // Forward LSA to other neighbors (except sender)
        forward_lsa_to_neighbors(lsa, sender_id)

Method should_accept_lsa(lsa: LinkStateAdvertisement) -> Boolean:
    if not link_state_database.contains(lsa.originator_id):
        return true  // New originator
    
    existing_lsa = link_state_database[lsa.originator_id]
    
    // Check sequence number for freshness
    if lsa.sequence_number > existing_lsa.sequence_number:
        return true  // Newer LSA
    
    return false  // Stale or duplicate

Method recompute_shortest_paths():
    // Use Dijkstra's algorithm on the link state database
    distances = new Map<String, Float>()
    previous = new Map<String, String>()
    unvisited = new Set<String>()
    
    // Initialize
    for each node_id in link_state_database.keys():
        distances[node_id] = INFINITY
        unvisited.add(node_id)
    
    distances[own_node_id] = 0.0
    
    while unvisited.size() > 0:
        // Find unvisited node with minimum distance
        current = find_minimum_distance_node(unvisited, distances)
        unvisited.remove(current)
        
        // Update distances to neighbors
        if link_state_database.contains(current):
            lsa = link_state_database[current]
            
            for each neighbor in lsa.neighbors:
                neighbor_id = neighbor.neighbor_id
                
                if unvisited.contains(neighbor_id):
                    link_cost = calculate_link_cost(neighbor)
                    alt_distance = distances[current] + link_cost
                    
                    if alt_distance < distances[neighbor_id]:
                        distances[neighbor_id] = alt_distance
                        previous[neighbor_id] = current
    
    // Update routing table with computed paths
    update_routing_table_from_shortest_paths(previous, distances)

Method calculate_link_cost(neighbor: NeighborInfo) -> Float:
    // Multi-factor link cost calculation
    rssi_cost = 1.0 - normalize_rssi(neighbor.rssi)
    quality_cost = 1.0 - neighbor.link_quality
    
    // Combine factors
    return rssi_cost * 0.6 + quality_cost * 0.4

Method update_routing_table_from_shortest_paths(previous: Map<String, String>, distances: Map<String, Float>):
    for each destination_id in previous.keys():
        if destination_id != own_node_id:
            // Find next hop by tracing back the path
            next_hop = find_next_hop_in_path(destination_id, previous)
            
            if next_hop != null:
                // Create or update route entry
                route_entry = new RouteEntry()
                route_entry.destination_id = destination_id
                route_entry.next_hop_id = next_hop
                route_entry.hop_count = calculate_hop_count_from_path(destination_id, previous)
                route_entry.route_creation_time = current_time()
                route_entry.route_expiry_time = current_time() + PROACTIVE_ROUTE_LIFETIME
                route_entry.route_flags = VALID | PROACTIVE_ROUTE
                
                routing_table.routes[destination_id] = route_entry
```

## Route Error Handling and Maintenance

```pseudocode
Method send_route_error(destination_id: String, destination_sequence_number: Integer):
    // Create RERR message for unreachable destination
    rerr = new RouteError()
    
    unreachable = new UnreachableDestination()
    unreachable.destination_id = destination_id
    unreachable.destination_sequence_number = destination_sequence_number + 1
    
    rerr.unreachable_destinations.add(unreachable)
    
    // Send RERR to precursors (nodes that might be using this route)
    route = routing_table.routes[destination_id]
    if route != null:
        for each precursor in route.precursor_list:
            send_rerr_to_precursor(rerr, precursor)
    
    // Invalidate the route
    invalidate_route(destination_id)

Method process_route_error(rerr: RouteError, sender_id: String):
    for each unreachable in rerr.unreachable_destinations:
        destination_id = unreachable.destination_id
        
        // Check if we have a route through the sender
        if routing_table.routes.contains(destination_id):
            route = routing_table.routes[destination_id]
            
            if route.next_hop_id == sender_id:
                // Our route is broken, try to repair or remove
                if can_repair_route_locally(route):
                    initiate_local_route_repair(destination_id)
                else:
                    invalidate_route(destination_id)
                    propagate_route_error(rerr, sender_id)

Method can_repair_route_locally(route: RouteEntry) -> Boolean:
    // Check if we have alternate paths
    if route.backup_routes.size() > 0:
        for each alternate in route.backup_routes:
            if is_neighbor_reachable(alternate.next_hop_id):
                return true
    
    // Check if we can use geographic routing as fallback
    if has_geographic_info(route.destination_id):
        return true
    
    return false

Method initiate_local_route_repair(destination_id: String):
    route = routing_table.routes[destination_id]
    
    // Try alternate routes first
    if route.backup_routes.size() > 0:
        best_alternate = find_best_alternate_route(route.backup_routes)
        if best_alternate != null and is_neighbor_reachable(best_alternate.next_hop_id):
            promote_alternate_to_primary(destination_id, best_alternate)
            return
    
    // Try geographic routing
    if has_geographic_info(destination_id):
        // Switch route to geographic mode
        route.route_flags |= GEOGRAPHIC_ROUTE
        return
    
    // Last resort: initiate new route discovery with limited scope
    initiate_limited_route_discovery(destination_id)

Method initiate_limited_route_discovery(destination_id: String):
    // Create RREQ with smaller TTL for local repair
    rreq = create_route_request(destination_id)
    rreq.ttl = min(rreq.ttl, LOCAL_REPAIR_TTL)  // Limit search scope
    
    // Mark as repair RREQ
    rreq.flags |= REPAIR_FLAG
    
    broadcast_rreq(rreq)

Method maintain_route_quality():
    // Periodically assess and update route quality metrics
    for each destination_id in routing_table.routes.keys():
        route = routing_table.routes[destination_id]
        
        // Update quality metrics
        update_route_quality_metrics(route)
        
        // Check if route needs improvement
        if route.route_quality_metrics.packet_loss_rate > MAX_ACCEPTABLE_LOSS_RATE:
            // Try to find better route
            if route.backup_routes.size() > 0:
                consider_route_switch(destination_id)
            else:
                initiate_route_improvement(destination_id)

Method update_route_quality_metrics(route: RouteEntry):
    next_hop_id = route.next_hop_id
    
    if neighbor_table.contains(next_hop_id):
        neighbor = neighbor_table[next_hop_id]
        
        // Update RSSI-based quality
        route.route_quality_metrics.average_rssi = 
            exponential_moving_average(route.route_quality_metrics.average_rssi, neighbor.rssi)
        
        // Update link quality
        current_quality = calculate_link_quality(neighbor)
        route.route_quality_metrics.stability_factor = 
            calculate_stability_factor(neighbor)
    
    // Update energy cost estimate
    route.route_quality_metrics.energy_cost = 
        estimate_route_energy_cost(route)

Method consider_route_switch(destination_id: String):
    current_route = routing_table.routes[destination_id]
    
    // Evaluate alternate routes
    for each alternate in current_route.backup_routes:
        alternate_quality = estimate_alternate_route_quality(alternate)
        current_quality = calculate_overall_quality_score(current_route)
        
        if alternate_quality > current_quality + ROUTE_SWITCH_THRESHOLD:
            // Switch to better alternate route
            promote_alternate_to_primary(destination_id, alternate)
            log("Switched to better route for " + destination_id)
            return

Method blacklist_problematic_neighbor(neighbor_id: String, duration: Duration):
    // Temporarily avoid using a problematic neighbor for routing
    blacklist_entry = new BlacklistEntry()
    blacklist_entry.neighbor_id = neighbor_id
    blacklist_entry.expiry_time = current_time() + duration
    blacklist_entry.reason = "Poor link quality or high packet loss"
    
    neighbor_blacklist[neighbor_id] = blacklist_entry
    
    // Remove routes through blacklisted neighbor
    invalidate_routes_through_neighbor(neighbor_id)

Method cleanup_blacklist():
    current_time = get_current_time()
    expired_entries = []
    
    for each neighbor_id in neighbor_blacklist.keys():
        entry = neighbor_blacklist[neighbor_id]
        
        if current_time > entry.expiry_time:
            expired_entries.add(neighbor_id)
    
    for each expired_id in expired_entries:
        neighbor_blacklist.remove(expired_id)
        log("Removed " + expired_id + " from blacklist")
```

```pseudocode
Method process_incoming_message(raw_message: Bytes):
    try:
        message = deserialize_message(raw_message)
        
        // Verify message signature
        if not verify_signature(message):
            log("Invalid signature, dropping message")
            return
        
        // Check if we've seen this message before (loop prevention)
        if message_cache.contains(message.message_id):
            return
        
        // Add to cache
        message_cache.add(message.message_id)
        
        // Update routing information from source
        update_routing_table_from_message(message)
        
        // Process based on message type
        switch message.message_type:
            case BEACON:
                process_beacon(message)
            case GPS_UPDATE:
                process_gps_update(message)
            case TEXT:
                process_text_message(message)
            case CONTROL:
                process_control_message(message)
            case ACK:
                process_acknowledgment(message)
        
        // Forward message if needed
        if should_forward_message(message):
            forward_message(message)
    
    catch Exception as e:
        log("Error processing message: " + e.message)
```

## Routing Table Management

```pseudocode
Method update_routing_table_from_message(message: Message):
    source_id = message.source_id
    
    // Update or create node entry
    if routing_table.nodes.contains(source_id):
        node = routing_table.nodes[source_id]
    else:
        node = new Node()
        node.node_id = source_id
    
    // Update node information
    node.last_seen = current_time()
    node.hop_count = message.hop_count
    node.rssi = lora_radio.get_last_rssi()
    
    // Determine if this is a direct neighbor (hop_count = 0 from source)
    node.is_direct_neighbor = (message.hop_count == 0)
    
    // Update routing table
    routing_table.nodes[source_id] = node
    
    // Update next hop information
    if node.is_direct_neighbor:
        routing_table.next_hop[source_id] = source_id
    else:
        // Find best route through neighbors
        update_route_to_node(source_id)

Method update_route_to_node(destination_id: String):
    best_route = null
    best_quality = -1
    
    // Check all direct neighbors for routes to destination
    for each neighbor in get_direct_neighbors():
        if neighbor.node_id == destination_id:
            // Direct route is always best
            routing_table.next_hop[destination_id] = neighbor.node_id
            return
        
        // Calculate route quality through this neighbor
        quality = calculate_route_quality(neighbor, destination_id)
        
        if quality > best_quality:
            best_quality = quality
            best_route = neighbor.node_id
    
    if best_route != null:
        routing_table.next_hop[destination_id] = best_route
        routing_table.route_quality[destination_id] = best_quality

Method calculate_route_quality(neighbor: Node, destination_id: String) -> Float:
    // Quality metric combines RSSI, hop count, and age
    rssi_factor = normalize_rssi(neighbor.rssi)  // 0.0 to 1.0
    hop_factor = 1.0 / (1.0 + neighbor.hop_count)  // Prefer fewer hops
    age_factor = calculate_age_factor(neighbor.last_seen)  // Prefer recent
    
    return rssi_factor * 0.4 + hop_factor * 0.4 + age_factor * 0.2

Method cleanup_routing_table():
    current_time = get_current_time()
    expired_nodes = []
    
    for each node_id in routing_table.nodes.keys():
        node = routing_table.nodes[node_id]
        age = current_time - node.last_seen
        
        if age > MAX_NODE_AGE:
            expired_nodes.add(node_id)
    
    for each expired_id in expired_nodes:
        routing_table.nodes.remove(expired_id)
        routing_table.next_hop.remove(expired_id)
        routing_table.route_quality.remove(expired_id)
```

## Message Forwarding Logic

```pseudocode
Method should_forward_message(message: Message) -> Boolean:
    // Don't forward if TTL expired
    if message.ttl <= 0:
        return false
    
    // Don't forward if we've already forwarded this message
    if message_cache.forwarded_messages.contains(message.message_id):
        return false
    
    // Don't forward if we're the destination
    if message.destination_id == own_node_id:
        return false
    
    // Don't forward if it's a broadcast and we've seen it
    if message.destination_id == "BROADCAST":
        return message.hop_count == 0  // Only forward direct broadcasts
    
    // Forward if we have a route to destination
    return routing_table.next_hop.contains(message.destination_id)

Method forward_message(message: Message):
    // Create forwarded copy
    forwarded_msg = copy_message(message)
    forwarded_msg.ttl = message.ttl - 1
    forwarded_msg.hop_count = message.hop_count + 1
    
    // Determine how to forward
    if message.destination_id == "BROADCAST":
        broadcast_message(forwarded_msg)
    else:
        route_message_to_destination(forwarded_msg)
    
    // Mark as forwarded
    message_cache.forwarded_messages.add(message.message_id)

Method route_message_to_destination(message: Message):
    destination = message.destination_id
    
    if routing_table.next_hop.contains(destination):
        next_hop = routing_table.next_hop[destination]
        send_message_to_neighbor(message, next_hop)
    else:
        // No route found, try to broadcast to direct neighbors
        log("No route to " + destination + ", broadcasting")
        broadcast_to_neighbors(message)
```

## Neighbor Discovery

```pseudocode
Method process_beacon(message: Message):
    // Beacon messages help discover and maintain neighbor relationships
    source_id = message.source_id
    
    // Extract neighbor information from beacon payload
    neighbor_info = deserialize_beacon_payload(message.payload)
    
    // Update our routing table
    update_neighbor_info(source_id, neighbor_info)
    
    // Send beacon reply if this is a new neighbor
    if is_new_neighbor(source_id):
        send_beacon_reply(source_id)

Method create_beacon_message() -> Message:
    beacon = new Message()
    beacon.message_id = generate_message_id()
    beacon.source_id = own_node_id
    beacon.destination_id = "BROADCAST"
    beacon.message_type = BEACON
    beacon.ttl = 1  // Beacons are not forwarded
    beacon.hop_count = 0
    beacon.timestamp = current_time()
    
    // Include our current status in payload
    payload = new BeaconPayload()
    payload.battery_level = get_battery_level()
    payload.gps_position = gps_module.get_position()
    payload.neighbor_count = get_direct_neighbor_count()
    
    beacon.payload = serialize(payload)
    beacon.signature = sign_message(beacon)
    
    return beacon
```

## GPS Message Handling

```pseudocode
Method process_gps_update(message: Message):
    source_id = message.source_id
    gps_data = deserialize_gps_payload(message.payload)
    
    // Update node position in routing table
    if routing_table.nodes.contains(source_id):
        node = routing_table.nodes[source_id]
        node.gps_position = gps_data.position
        node.last_seen = current_time()
    
    // Notify application layer about position update
    notify_position_update(source_id, gps_data.position)
    
    // Log for debugging
    log("GPS update from " + source_id + " at " + gps_data.position.toString())

Method send_gps_update():
    if not gps_module.has_fix():
        return
    
    gps_msg = new Message()
    gps_msg.message_id = generate_message_id()
    gps_msg.source_id = own_node_id
    gps_msg.destination_id = "BROADCAST"
    gps_msg.message_type = GPS_UPDATE
    gps_msg.ttl = MAX_TTL
    gps_msg.hop_count = 0
    gps_msg.timestamp = current_time()
    
    payload = new GPSPayload()
    payload.position = gps_module.get_position()
    payload.accuracy = gps_module.get_accuracy()
    payload.timestamp = gps_module.get_timestamp()
    
    gps_msg.payload = serialize(payload)
    gps_msg.signature = sign_message(gps_msg)
    
    broadcast_message(gps_msg)
```

## Adaptive Routing Features

```pseudocode
Method adaptive_transmission_power():
    // Adjust transmission power based on network density
    neighbor_count = get_direct_neighbor_count()
    
    if neighbor_count < 2:
        // Increase power to find more neighbors
        lora_radio.set_transmission_power(MAX_POWER)
    else if neighbor_count > 8:
        // Reduce power to decrease interference
        lora_radio.set_transmission_power(MIN_POWER)
    else:
        // Use medium power
        lora_radio.set_transmission_power(MEDIUM_POWER)

Method adaptive_beacon_interval():
    // Adjust beacon frequency based on mobility and network stability
    movement_speed = calculate_movement_speed()
    network_stability = calculate_network_stability()
    
    base_interval = 30  // seconds
    
    if movement_speed > HIGH_SPEED_THRESHOLD:
        // Moving fast, beacon more frequently
        return base_interval / 2
    else if network_stability > STABLE_THRESHOLD:
        // Network is stable, beacon less frequently
        return base_interval * 2
    else:
        return base_interval

Method calculate_movement_speed() -> Float:
    // Calculate speed based on recent GPS positions
    if gps_position_history.size() < 2:
        return 0.0
    
    recent_positions = gps_position_history.get_last_n(5)
    total_distance = 0.0
    total_time = 0.0
    
    for i = 1 to recent_positions.size() - 1:
        distance = calculate_distance(recent_positions[i-1], recent_positions[i])
        time_diff = recent_positions[i].timestamp - recent_positions[i-1].timestamp
        
        total_distance += distance
        total_time += time_diff
    
    if total_time > 0:
        return total_distance / total_time
    else:
        return 0.0
```

## Message Prioritization

```pseudocode
Method prioritize_message_queue():
    // Sort outbound messages by priority
    message_queue.sort_by(message -> calculate_message_priority(message))

Method calculate_message_priority(message: Message) -> Integer:
    base_priority = 0
    
    switch message.message_type:
        case CONTROL:
            base_priority = 100  // Highest priority
        case ACK:
            base_priority = 80
        case GPS_UPDATE:
            base_priority = 60
        case TEXT:
            base_priority = 40
        case BEACON:
            base_priority = 20   // Lowest priority
    
    // Adjust based on message age
    age = current_time() - message.timestamp
    age_penalty = min(age / 10, 50)  // Max 50 point penalty
    
    return base_priority - age_penalty

Method congestion_control():
    // Implement backoff when channel is busy
    if channel_busy_ratio > HIGH_CONGESTION_THRESHOLD:
        // Exponential backoff
        current_backoff = min(current_backoff * 2, MAX_BACKOFF)
    else if channel_busy_ratio < LOW_CONGESTION_THRESHOLD:
        // Reduce backoff
        current_backoff = max(current_backoff / 2, MIN_BACKOFF)
    
    delay(current_backoff)
```

## Error Handling and Recovery

```pseudocode
Method handle_transmission_failure(message: Message):
    retry_count = get_retry_count(message.message_id)
    
    if retry_count < MAX_RETRIES:
        // Exponential backoff retry
        delay = INITIAL_RETRY_DELAY * (2 ^ retry_count)
        schedule_retry(message, delay)
        increment_retry_count(message.message_id)
    else:
        // Give up and report failure
        log("Failed to deliver message " + message.message_id + " after " + MAX_RETRIES + " attempts")
        notify_delivery_failure(message)

Method detect_network_partition():
    // Detect if we're isolated from the main network
    recent_messages = get_recent_message_count(last_5_minutes)
    active_neighbors = get_active_neighbor_count()
    
    if recent_messages < MIN_EXPECTED_MESSAGES and active_neighbors < MIN_NEIGHBORS:
        log("Possible network partition detected")
        enter_partition_recovery_mode()

Method enter_partition_recovery_mode():
    // Increase beacon frequency and transmission power to reconnect
    beacon_interval = PARTITION_BEACON_INTERVAL  // Faster beacons
    lora_radio.set_transmission_power(MAX_POWER)
    
    // Start scanning for networks on different frequencies
    start_frequency_scanning()
```

## Advanced Routing Features

```pseudocode
Method analyze_network_topology():
    // Perform periodic network topology analysis for optimization
    
    topology_metrics = calculate_topology_metrics()
    
    // Detect network partitions
    partitions = detect_network_partitions()
    if partitions.size() > 1:
        handle_network_partition(partitions)
    
    // Identify critical nodes (bottlenecks)
    critical_nodes = identify_critical_nodes(topology_metrics)
    optimize_routes_around_critical_nodes(critical_nodes)
    
    // Update routing protocol weights based on network state
    update_routing_protocol_weights()

Method calculate_topology_metrics() -> TopologyMetrics:
    metrics = new TopologyMetrics()
    
    // Calculate network density
    total_nodes = link_state_database.size()
    total_links = count_total_links()
    metrics.network_density = total_links / (total_nodes * (total_nodes - 1) / 2)
    
    // Calculate average path length
    metrics.average_path_length = calculate_average_path_length()
    
    // Calculate clustering coefficient
    metrics.clustering_coefficient = calculate_clustering_coefficient()
    
    // Calculate network diameter
    metrics.network_diameter = calculate_network_diameter()
    
    return metrics

Method implement_load_balancing():
    // Distribute traffic across multiple paths to prevent bottlenecks
    
    for each destination_id in routing_table.routes.keys():
        route = routing_table.routes[destination_id]
        
        // Check if route is heavily loaded
        if is_route_overloaded(route):
            // Try to distribute load to alternate paths
            distribute_load_to_alternates(destination_id, route)

Method distribute_load_to_alternates(destination_id: String, primary_route: RouteEntry):
    if primary_route.backup_routes.size() == 0:
        return  // No alternates available
    
    // Calculate load distribution ratios
    total_capacity = calculate_total_route_capacity(primary_route)
    
    for each alternate in primary_route.backup_routes:
        if is_neighbor_reachable(alternate.next_hop_id):
            // Assign traffic percentage based on route quality
            traffic_percentage = alternate.quality_score / total_capacity
            assign_traffic_to_route(destination_id, alternate.next_hop_id, traffic_percentage)

Method implement_qos_routing():
    // Quality of Service routing for different message types
    
    // Define QoS classes
    qos_classes = [
        new QoSClass("CRITICAL", priority: 100, max_delay: 1000ms, min_reliability: 0.99),
        new QoSClass("HIGH", priority: 80, max_delay: 3000ms, min_reliability: 0.95),
        new QoSClass("NORMAL", priority: 60, max_delay: 10000ms, min_reliability: 0.90),
        new QoSClass("BEST_EFFORT", priority: 20, max_delay: INFINITY, min_reliability: 0.70)
    ]

Method route_with_qos_constraints(message: Message, qos_requirements: QoSRequirements) -> Boolean:
    destination_id = message.destination_id
    
    // Find routes that meet QoS requirements
    suitable_routes = find_routes_meeting_qos(destination_id, qos_requirements)
    
    if suitable_routes.size() == 0:
        // No suitable routes, try best effort
        return route_message(message)
    
    // Select best route based on QoS metrics
    best_route = select_best_qos_route(suitable_routes, qos_requirements)
    
    return forward_message_via_route(message, best_route)

Method find_routes_meeting_qos(destination_id: String, qos_requirements: QoSRequirements) -> List<RouteEntry>:
    suitable_routes = []
    
    if routing_table.routes.contains(destination_id):
        primary_route = routing_table.routes[destination_id]
        
        if route_meets_qos_requirements(primary_route, qos_requirements):
            suitable_routes.add(primary_route)
        
        // Check alternate routes
        for each alternate in primary_route.backup_routes:
            alternate_route = create_route_entry_from_alternate(alternate, destination_id)
            if route_meets_qos_requirements(alternate_route, qos_requirements):
                suitable_routes.add(alternate_route)
    
    return suitable_routes

Method implement_energy_aware_routing():
    // Route messages considering energy constraints of nodes
    
    for each destination_id in routing_table.routes.keys():
        route = routing_table.routes[destination_id]
        
        // Check energy levels of nodes in the path
        path_energy_level = calculate_path_energy_level(route)
        
        if path_energy_level < CRITICAL_ENERGY_THRESHOLD:
            // Find alternative path to preserve energy
            energy_efficient_route = find_energy_efficient_route(destination_id)
            
            if energy_efficient_route != null:
                replace_route_with_energy_efficient(destination_id, energy_efficient_route)

Method calculate_path_energy_level(route: RouteEntry) -> Float:
    // Estimate remaining energy along the route path
    
    if not neighbor_table.contains(route.next_hop_id):
        return 1.0  // Assume full energy if unknown
    
    next_hop = neighbor_table[route.next_hop_id]
    
    // Use next hop's battery level as proxy for path energy
    path_energy = next_hop.battery_level / 100.0
    
    // Apply energy cost factor based on route length
    energy_cost_factor = 1.0 - (route.hop_count * ENERGY_COST_PER_HOP)
    
    return path_energy * energy_cost_factor

Method implement_mobility_adaptive_routing():
    // Adapt routing strategies based on node mobility patterns
    
    mobility_level = calculate_network_mobility_level()
    
    if mobility_level > HIGH_MOBILITY_THRESHOLD:
        // High mobility: prefer geographic routing
        increase_geographic_routing_weight()
        reduce_proactive_routing_frequency()
        increase_reactive_routing_responsiveness()
    else if mobility_level < LOW_MOBILITY_THRESHOLD:
        // Low mobility: prefer proactive routing
        increase_proactive_routing_weight()
        reduce_geographic_routing_weight()
        decrease_route_discovery_frequency()

Method calculate_network_mobility_level() -> Float:
    total_speed = 0.0
    node_count = 0
    
    for each node_id in neighbor_table.keys():
        node = neighbor_table[node_id]
        speed = calculate_node_speed(node)
        
        total_speed += speed
        node_count += 1
    
    if node_count > 0:
        return total_speed / node_count
    else:
        return 0.0
```

## Configuration Constants

These constants define the behavior and performance characteristics of the routing system. They should be tuned based on specific deployment requirements and network conditions.

```pseudocode
Constants:
    // === Basic Routing Parameters ===
    MAX_TTL = 15                           // Maximum hops for a message
                                          // Higher values increase reach but also network load
    
    MAX_NODE_AGE = 300                     // Seconds before considering node offline
                                          // Balance between stale info and rapid updates
    
    MESSAGE_CACHE_SIZE = 2000              // Maximum cached message IDs for duplicate detection
                                          // Must be large enough to handle network diameter
    
    MAX_RETRIES = 3                        // Maximum transmission retries per message
                                          // Higher values improve reliability but increase delay
    
    // === Route Discovery Parameters ===
    RREQ_TIMEOUT = 10000                   // RREQ timeout in milliseconds
                                          // Time to wait for route reply before giving up
    
    RREQ_CACHE_TIMEOUT = 30                // RREQ cache timeout in seconds
                                          // Prevents processing duplicate requests
    
    MAX_RREQ_HOPS = 10                     // Maximum RREQ propagation hops
                                          // Limits flooding scope for route discovery
    
    LOCAL_REPAIR_TTL = 3                   // TTL for local route repair attempts
                                          // Small value for localized repair efforts
    
    REVERSE_ROUTE_LIFETIME = 10000         // Reverse route lifetime in milliseconds
                                          // Temporary routes for RREP delivery
    
    ACTIVE_ROUTE_LIFETIME = 30000          // Active route lifetime in milliseconds
                                          // How long established routes remain valid
    
    PROACTIVE_ROUTE_LIFETIME = 120000      // Proactive route lifetime in milliseconds
                                          // Longer lifetime for link-state based routes
    
    // === Multi-Path Routing Parameters ===
    MAX_ALTERNATE_ROUTES = 3               // Maximum alternate routes per destination
                                          // Balance between options and memory usage
    
    MULTIPATH_RREQ_DELAY = 100             // Delay between multipath RREQs in milliseconds
                                          // Staggers requests to reduce collisions
    
    ROUTE_SWITCH_THRESHOLD = 0.2           // Quality improvement threshold for route switching
                                          // Prevents excessive route oscillation
    
    // === Geographic Routing Parameters ===
    MIN_GEOGRAPHIC_LINK_QUALITY = 0.3     // Minimum link quality for geographic routing
                                          // Ensures reliable forwarding in geo mode
    
    VELOCITY_CALCULATION_WINDOW = 30       // Window for velocity calculation in seconds
                                          // Smooths mobility estimation over time
    
    PREDICTION_TIME_HORIZON = 10           // Time horizon for position prediction in seconds
                                          // How far ahead to predict node movement
    
    AVERAGE_RADIO_RANGE = 1000            // Average radio range in meters
                                          // Used for geographic distance estimates
    
    METERS_PER_DEGREE_LAT = 111000        // Meters per degree latitude
                                          // Geographic coordinate conversion constant
    
    METERS_PER_DEGREE_LON = 85000         // Meters per degree longitude (approximate)
                                          // Varies with latitude but good average
    
    // === Link State Parameters ===
    LSA_INTERVAL = 60                      // Link State Advertisement interval in seconds
                                          // Balance between topology freshness and overhead
    
    NEIGHBOR_HELLO_INTERVAL = 10           // Hello message interval in seconds
                                          // Frequency of neighbor discovery messages
    
    // === Quality of Service Parameters ===
    CRITICAL_ENERGY_THRESHOLD = 0.2       // Critical energy level threshold (20%)
                                          // When to avoid energy-constrained nodes
    
    ENERGY_COST_PER_HOP = 0.05            // Energy cost factor per hop (5% per hop)
                                          // Estimate of battery drain per forwarding operation
    
    // === Mobility Parameters ===
    HIGH_MOBILITY_THRESHOLD = 10.0         // High mobility speed threshold in m/s (36 km/h)
                                          // Above this, prefer geographic routing
    
    LOW_MOBILITY_THRESHOLD = 1.0           // Low mobility speed threshold in m/s (3.6 km/h)
                                          // Below this, prefer proactive routing
    
    // === Network Analysis Parameters ===
    TOPOLOGY_ANALYSIS_INTERVAL = 120       // Topology analysis interval in seconds
                                          // How often to analyze network structure
    
    ROUTE_CLEANUP_INTERVAL = 60            // Route cleanup interval in seconds
                                          // Frequency of routing table maintenance
    
    GPS_UPDATE_INTERVAL = 10               // GPS update broadcast interval in seconds
                                          // How often nodes share position updates
    
    // === Radio Transmission Parameters ===
    MAX_POWER = 20                         // Maximum transmission power in dBm
                                          // Highest power for maximum range
    
    MIN_POWER = 2                          // Minimum transmission power in dBm
                                          // Lowest power for energy conservation
    
    MEDIUM_POWER = 14                      // Medium transmission power in dBm
                                          // Balanced power for normal operation
    
    // === Congestion Control Parameters ===
    HIGH_CONGESTION_THRESHOLD = 0.8        // High congestion threshold (80% channel busy)
                                          // Above this, implement aggressive backoff
    
    LOW_CONGESTION_THRESHOLD = 0.3         // Low congestion threshold (30% channel busy)
                                          // Below this, reduce backoff times
    
    MAX_BACKOFF = 5000                     // Maximum backoff time in milliseconds
                                          // Upper limit for congestion avoidance delay
    
    MIN_BACKOFF = 100                      // Minimum backoff time in milliseconds
                                          // Lower limit for transmission delays
    
    INITIAL_RETRY_DELAY = 1000            // Initial retry delay in milliseconds
                                          // Starting point for exponential backoff
    
    // === Quality Thresholds ===
    MAX_ACCEPTABLE_LOSS_RATE = 0.1         // Maximum acceptable packet loss rate (10%)
                                          // Above this, consider route problematic
    
    MIN_ACCEPTABLE_RSSI = -90              // Minimum acceptable RSSI in dBm
                                          // Below this, consider link poor quality
    
    ROUTE_QUALITY_UPDATE_INTERVAL = 30     // Route quality update interval in seconds
                                          // How often to recalculate route metrics
    
    // === Multiple Receiver Coordination Parameters ===
    HIGH_DENSITY_THRESHOLD = 8             // Node count threshold for dense network detection
                                          // Above this, implement aggressive forwarding suppression
    
    MIN_FORWARDING_DISTANCE = 100          // Minimum distance in meters before forwarding
                                          // Prevents nearby nodes from redundant forwarding
    
    FORWARDING_MONITORING_WINDOW = 2000    // Milliseconds to monitor for duplicate transmissions
                                          // Window to detect if others have forwarded message
    
    MAX_GEOGRAPHIC_FORWARDING_DELAY = 1000 // Maximum delay for geographic forwarding in milliseconds
                                          // Upper bound for collision avoidance delays
    
    RREQ_COORDINATION_DELAY_RANGE = 500    // Maximum delay range for RREQ coordination in milliseconds
                                          // Prevents RREQ flooding storms
    
    BROADCAST_SUPPRESSION_THRESHOLD = 0.7  // Threshold for broadcast suppression (70%)
                                          // If forwarding score below this, suppress broadcast
    
    EDGE_NODE_DETECTION_RADIUS = 2000      // Radius in meters for edge node detection
                                          // Nodes with fewer neighbors in this radius are "edge"
    
    // === Advanced Feature Parameters ===
    LOW_DENSITY = 0.3                      // Low network density threshold
                                          // Below this, favor geographic routing
    
    STABLE_THRESHOLD = 0.8                 // Network stability threshold
                                          // Above this, reduce beacon frequency
    
    MIN_NEIGHBORS = 2                      // Minimum neighbors before partition detection
                                          // Below this, assume network isolation
    
    PARTITION_BEACON_INTERVAL = 5          // Beacon interval during partition recovery
                                          // Faster beacons when network partitioned

/**
 * Configuration Guidelines:
 * 
 * For Dense Networks (>20 nodes in radio range):
 * - Reduce LSA_INTERVAL to 30s for faster convergence
 * - Increase MIN_POWER to 10 dBm to reduce interference
 * - Set MAX_ALTERNATE_ROUTES to 5 for better redundancy
 * 
 * For Sparse Networks (<5 nodes in radio range):
 * - Increase MAX_POWER to 20 dBm for maximum range
 * - Reduce NEIGHBOR_HELLO_INTERVAL to 5s for faster discovery
 * - Set MIN_GEOGRAPHIC_LINK_QUALITY to 0.2 for more options
 * 
 * For High Mobility (vehicles, aircraft):
 * - Reduce ACTIVE_ROUTE_LIFETIME to 15000ms
 * - Increase GPS_UPDATE_INTERVAL frequency to 5s
 * - Set PREDICTION_TIME_HORIZON to 15s for better prediction
 * 
 * For Low Power Applications:
 * - Increase all intervals by 2x to reduce radio activity
 * - Set CRITICAL_ENERGY_THRESHOLD to 0.3 (30%)
 * - Use MIN_POWER as default transmission power
 * 
 * For Mission-Critical Applications:
 * - Reduce MAX_ACCEPTABLE_LOSS_RATE to 0.05 (5%)
 * - Increase MAX_ALTERNATE_ROUTES to 5
 * - Set MAX_RETRIES to 5 for better reliability
 */
```

```

This enhanced pseudocode provides a comprehensive and sophisticated foundation for implementing a robust LoRa mesh routing system with multiple routing protocols, advanced route discovery, geographic routing, link state management, QoS support, and intelligent route maintenance for the GPS Team Awareness Kit.

## Key Features Implemented:

### 1. **AODV Route Discovery Protocol**
- Route Request (RREQ) and Route Reply (RREP) with sequence numbers
- Loop prevention and duplicate detection
- Fresh route discovery with hop counting

### 2. **Multi-Path Routing**
- Discovery and maintenance of alternate routes
- Load balancing across multiple paths
- Route quality-based path selection

### 3. **Geographic Routing**
- GPS-coordinate based forwarding
- Greedy forwarding with perimeter routing fallback  
- Predictive routing for mobile scenarios
- Mobility-aware route optimization

### 4. **Link State Protocol**
- Proactive topology maintenance
- Dijkstra's shortest path computation
- Network-wide topology awareness

### 5. **Advanced Route Maintenance**
- Route error handling (RERR messages)
- Local route repair mechanisms
- Neighbor blacklisting for problematic links
- Quality-based route switching

### 6. **Quality of Service (QoS)**
- Message prioritization
- QoS-constrained routing
- Energy-aware path selection
- Congestion control

### 7. **Adaptive Features**
- Mobility-adaptive protocol selection
- Network topology analysis
- Dynamic transmission power adjustment
- Protocol weight optimization

This routing system is designed to handle complex scenarios including high mobility, network partitions, varying link quality, energy constraints, and different QoS requirements while maintaining efficiency and reliability in a LoRa mesh network environment.