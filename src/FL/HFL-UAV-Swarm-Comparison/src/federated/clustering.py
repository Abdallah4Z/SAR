"""
Clustering utilities for Hierarchical Federated Learning.

This module provides various clustering strategies for grouping UAVs
into clusters for hierarchical aggregation:
    - K-Means: Position-based clustering
    - Proximity-Based: Communication range clustering
    - Round-Robin: Simple sequential assignment
    - Dynamic Manager: Adaptive re-clustering based on movement
"""

import numpy as np
from typing import List, Dict, Tuple, Optional
from collections import defaultdict


class KMeansClustering:
    """
    K-Means clustering for UAV positions.
    
    Uses numpy-only implementation with k-means++ initialization
    for stable centroid placement. No sklearn dependency.
    
    Args:
        n_clusters: Number of clusters to form
        max_iters: Maximum iterations for convergence (default: 100)
        random_state: Random seed for reproducibility
    """
    
    def __init__(
        self,
        n_clusters: int,
        max_iters: int = 100,
        random_state: Optional[int] = None
    ):
        self.n_clusters = n_clusters
        self.max_iters = max_iters
        self.random_state = random_state
        self.centroids = None
        
        if random_state is not None:
            np.random.seed(random_state)
    
    def _kmeans_plus_plus_init(self, positions: np.ndarray) -> np.ndarray:
        """
        K-means++ initialization for better centroid placement.
        
        Selects initial centroids with probability proportional to
        distance from nearest existing centroid.
        
        Args:
            positions: Agent positions (n_agents, 3)
        
        Returns:
            centroids: Initial centroids (n_clusters, 3)
        """
        n_samples = positions.shape[0]
        centroids = np.zeros((self.n_clusters, positions.shape[1]))
        
        # Choose first centroid randomly
        centroids[0] = positions[np.random.randint(n_samples)]
        
        # Choose remaining centroids
        for k in range(1, self.n_clusters):
            # Compute distances to nearest centroid
            distances = np.min([
                np.sum((positions - centroids[j])**2, axis=1)
                for j in range(k)
            ], axis=0)
            
            # Choose next centroid with probability proportional to distance^2
            probabilities = distances / distances.sum()
            cumulative_probs = np.cumsum(probabilities)
            r = np.random.rand()
            
            for idx, cum_prob in enumerate(cumulative_probs):
                if r < cum_prob:
                    centroids[k] = positions[idx]
                    break
        
        return centroids
    
    def fit_predict(self, positions: np.ndarray) -> np.ndarray:
        """
        Perform K-means clustering on positions.
        
        Args:
            positions: Agent positions (n_agents, 3) or (n_agents, 2)
        
        Returns:
            labels: Cluster assignments (n_agents,)
        """
        if positions.shape[0] < self.n_clusters:
            # Not enough agents: assign each to separate cluster
            return np.arange(positions.shape[0])
        
        # Initialize centroids with k-means++
        self.centroids = self._kmeans_plus_plus_init(positions)
        
        labels = np.zeros(positions.shape[0], dtype=int)
        
        # Iterate until convergence
        for iteration in range(self.max_iters):
            # Assign each point to nearest centroid
            distances = np.array([
                np.sum((positions - self.centroids[k])**2, axis=1)
                for k in range(self.n_clusters)
            ]).T
            new_labels = np.argmin(distances, axis=1)
            
            # Check convergence
            if np.all(labels == new_labels):
                break
            
            labels = new_labels
            
            # Update centroids
            for k in range(self.n_clusters):
                cluster_points = positions[labels == k]
                if len(cluster_points) > 0:
                    self.centroids[k] = cluster_points.mean(axis=0)
        
        return labels


class ProximityBasedClustering:
    """
    Proximity-based clustering using communication range.
    
    Groups UAVs that are within communication range of each other
    into the same cluster. Uses a greedy graph-based approach.
    
    Args:
        communication_range: Maximum distance for same cluster (meters)
    """
    
    def __init__(self, communication_range: float = 50.0):
        self.communication_range = communication_range
    
    def fit_predict(self, positions: np.ndarray) -> np.ndarray:
        """
        Cluster agents based on proximity.
        
        Args:
            positions: Agent positions (n_agents, 3) or (n_agents, 2)
        
        Returns:
            labels: Cluster assignments (n_agents,)
        """
        n_agents = positions.shape[0]
        
        # Compute pairwise distances
        distances = np.zeros((n_agents, n_agents))
        for i in range(n_agents):
            for j in range(i + 1, n_agents):
                dist = np.linalg.norm(positions[i] - positions[j])
                distances[i, j] = dist
                distances[j, i] = dist
        
        # Build adjacency matrix (within communication range)
        adjacency = distances <= self.communication_range
        
        # Greedy clustering using DFS
        labels = -np.ones(n_agents, dtype=int)
        cluster_id = 0
        
        for agent_id in range(n_agents):
            if labels[agent_id] == -1:
                # Start new cluster
                self._dfs_cluster(agent_id, cluster_id, adjacency, labels)
                cluster_id += 1
        
        return labels
    
    def _dfs_cluster(
        self,
        agent_id: int,
        cluster_id: int,
        adjacency: np.ndarray,
        labels: np.ndarray
    ):
        """
        Depth-first search to assign cluster.
        
        Args:
            agent_id: Current agent
            cluster_id: Cluster to assign
            adjacency: Adjacency matrix
            labels: Cluster labels (modified in-place)
        """
        labels[agent_id] = cluster_id
        
        # Visit all neighbors
        for neighbor_id in range(len(labels)):
            if labels[neighbor_id] == -1 and adjacency[agent_id, neighbor_id]:
                self._dfs_cluster(neighbor_id, cluster_id, adjacency, labels)


class RoundRobinClustering:
    """
    Simple round-robin cluster assignment.
    
    Assigns agents to clusters sequentially: agent i → cluster i % n_clusters.
    Used as fallback when position data is unavailable.
    
    Args:
        n_clusters: Number of clusters
    """
    
    def __init__(self, n_clusters: int):
        self.n_clusters = n_clusters
    
    def fit_predict(self, n_agents: int) -> np.ndarray:
        """
        Assign agents to clusters in round-robin fashion.
        
        Args:
            n_agents: Number of agents
        
        Returns:
            labels: Cluster assignments (n_agents,)
        """
        labels = np.array([i % self.n_clusters for i in range(n_agents)])
        return labels


class DynamicClusterManager:
    """
    Dynamic cluster manager with adaptive re-clustering.
    
    Manages clustering over time, deciding when to re-cluster based on:
        - Update interval: Re-cluster every N rounds
        - Movement threshold: Re-cluster if agents moved significantly
    
    Args:
        strategy: Clustering strategy ('kmeans', 'proximity', 'roundrobin')
        n_clusters: Number of clusters ('auto' or integer)
        update_interval: Re-cluster every N rounds (default: 10)
        movement_threshold: Re-cluster if avg movement > threshold (meters)
    """
    
    def __init__(
        self,
        strategy: str = 'kmeans',
        n_clusters: str = 'auto',
        update_interval: int = 10,
        movement_threshold: float = 50.0,
        communication_range: float = 50.0
    ):
        self.strategy = strategy
        self.n_clusters = n_clusters
        self.update_interval = update_interval
        self.movement_threshold = movement_threshold
        self.communication_range = communication_range
        
        # State
        self.current_labels = None
        self.last_positions = None
        self.last_update_round = 0
        self.update_history = []
        
        # Clustering algorithm
        self.clusterer = None
    
    def _determine_n_clusters(self, n_agents: int) -> int:
        """
        Determine number of clusters automatically.
        
        Heuristic: Use sqrt(n_agents) clusters, but at least 2 and at most n_agents//2.
        
        Args:
            n_agents: Number of agents
        
        Returns:
            int: Number of clusters
        """
        if isinstance(self.n_clusters, int):
            return min(self.n_clusters, n_agents)
        
        # Auto determination
        n = max(2, min(int(np.sqrt(n_agents)), n_agents // 2))
        return n
    
    def update(
        self,
        agent_positions: Dict[int, np.ndarray],
        current_round: int
    ) -> Tuple[Dict[int, int], bool]:
        """
        Update clustering based on current agent positions.
        
        Decides whether to re-cluster based on:
            - Time since last update (update_interval)
            - Significant movement (movement_threshold)
        
        Args:
            agent_positions: Dict mapping agent_id to position [x, y, z]
            current_round: Current training round
        
        Returns:
            assignments: Dict mapping agent_id to cluster_id
            changed: True if clusters were updated
        """
        # Convert positions to array
        agent_ids = sorted(agent_positions.keys())
        positions = np.array([agent_positions[aid] for aid in agent_ids])
        n_agents = len(agent_ids)
        
        # Determine number of clusters
        n_clusters = self._determine_n_clusters(n_agents)
        
        # Check if re-clustering is needed
        should_recluster = False
        
        if self.current_labels is None:
            # First time: always cluster
            should_recluster = True
            reason = "initial"
        elif current_round - self.last_update_round >= self.update_interval:
            # Periodic update
            should_recluster = True
            reason = "interval"
        elif self.last_positions is not None:
            # Check movement threshold
            avg_movement = np.mean(np.linalg.norm(
                positions - self.last_positions, axis=1
            ))
            if avg_movement > self.movement_threshold:
                should_recluster = True
                reason = f"movement_{avg_movement:.1f}m"
        
        if should_recluster:
            # Perform clustering
            if self.strategy == 'kmeans':
                self.clusterer = KMeansClustering(n_clusters=n_clusters)
                labels = self.clusterer.fit_predict(positions)
            elif self.strategy == 'proximity':
                self.clusterer = ProximityBasedClustering(
                    communication_range=self.communication_range
                )
                labels = self.clusterer.fit_predict(positions)
            elif self.strategy == 'roundrobin':
                self.clusterer = RoundRobinClustering(n_clusters=n_clusters)
                labels = self.clusterer.fit_predict(n_agents)
            else:
                raise ValueError(f"Unknown strategy: {self.strategy}")
            
            self.current_labels = labels
            self.last_positions = positions.copy()
            self.last_update_round = current_round
            
            # Log update
            self.update_history.append({
                'round': current_round,
                'reason': reason,
                'n_clusters': len(np.unique(labels)),
            })
        
        # Convert to dictionary
        assignments = {agent_ids[i]: int(self.current_labels[i]) for i in range(n_agents)}
        
        return assignments, should_recluster
    
    def get_cluster_assignments(self) -> Dict[int, List[int]]:
        """
        Get current cluster assignments.
        
        Returns:
            dict: Mapping from cluster_id to list of agent_ids
        """
        if self.current_labels is None:
            return {}
        
        clusters = defaultdict(list)
        for agent_id, cluster_id in enumerate(self.current_labels):
            clusters[cluster_id].append(agent_id)
        
        return dict(clusters)
    
    def get_update_history(self) -> List[Dict]:
        """
        Get history of clustering updates.
        
        Returns:
            list: History of update events
        """
        return self.update_history.copy()


if __name__ == '__main__':
    """Smoke test for clustering algorithms."""
    
    print("Testing clustering algorithms...\n")
    
    # Test KMeansClustering
    print("Test 1: K-Means Clustering")
    positions = np.array([
        [0, 0, 50],
        [1, 1, 50],
        [100, 100, 50],
        [101, 101, 50],
        [200, 200, 50],
        [201, 201, 50],
    ])
    
    kmeans = KMeansClustering(n_clusters=3, random_state=42)
    labels = kmeans.fit_predict(positions)
    
    assert len(labels) == len(positions), f"Expected {len(positions)} labels, got {len(labels)}"
    assert len(np.unique(labels)) <= 3, f"Expected at most 3 clusters, got {len(np.unique(labels))}"
    print(f"✓ K-Means: {len(positions)} agents → {len(np.unique(labels))} clusters")
    print(f"  Labels: {labels}")
    print(f"  Centroids shape: {kmeans.centroids.shape}")
    
    # Test ProximityBasedClustering
    print("\nTest 2: Proximity-Based Clustering")
    proximity = ProximityBasedClustering(communication_range=50.0)
    labels_prox = proximity.fit_predict(positions)
    
    assert len(labels_prox) == len(positions), f"Expected {len(positions)} labels"
    print(f"✓ Proximity: {len(positions)} agents → {len(np.unique(labels_prox))} clusters")
    print(f"  Labels: {labels_prox}")
    
    # Test RoundRobinClustering
    print("\nTest 3: Round-Robin Clustering")
    roundrobin = RoundRobinClustering(n_clusters=3)
    labels_rr = roundrobin.fit_predict(n_agents=10)
    
    assert len(labels_rr) == 10, f"Expected 10 labels, got {len(labels_rr)}"
    assert len(np.unique(labels_rr)) == 3, f"Expected 3 clusters, got {len(np.unique(labels_rr))}"
    print(f"✓ Round-Robin: 10 agents → {len(np.unique(labels_rr))} clusters")
    print(f"  Labels: {labels_rr}")
    
    # Test DynamicClusterManager
    print("\nTest 4: Dynamic Cluster Manager")
    manager = DynamicClusterManager(
        strategy='kmeans',
        n_clusters='auto',
        update_interval=5,
        movement_threshold=50.0
    )
    
    # Initial clustering
    agent_positions = {i: positions[i] for i in range(len(positions))}
    assignments, changed = manager.update(agent_positions, current_round=0)
    
    assert changed is True, "First update should trigger clustering"
    assert len(assignments) == len(positions), f"Expected {len(positions)} assignments"
    print(f"✓ Dynamic Manager: Initial clustering")
    print(f"  Assignments: {assignments}")
    print(f"  Changed: {changed}")
    
    # Update without significant movement
    assignments2, changed2 = manager.update(agent_positions, current_round=1)
    assert changed2 is False, "No significant change, should not re-cluster"
    print(f"✓ No re-clustering (no movement)")
    
    # Update with movement
    moved_positions = {i: positions[i] + np.array([100, 0, 0]) for i in range(len(positions))}
    assignments3, changed3 = manager.update(moved_positions, current_round=2)
    assert changed3 is True, "Significant movement should trigger re-clustering"
    print(f"✓ Re-clustering triggered by movement")
    
    # Test periodic update
    for round_num in range(3, 8):
        _, changed = manager.update(moved_positions, current_round=round_num)
    assignments4, changed4 = manager.update(moved_positions, current_round=8)
    print(f"✓ Periodic update after interval: changed={changed4}")
    
    # Get cluster assignments
    cluster_dict = manager.get_cluster_assignments()
    print(f"✓ Cluster assignments: {len(cluster_dict)} clusters")
    for cluster_id, agent_ids in cluster_dict.items():
        print(f"  Cluster {cluster_id}: {len(agent_ids)} agents")
    
    # Get update history
    history = manager.get_update_history()
    print(f"✓ Update history: {len(history)} updates")
    for entry in history:
        print(f"  Round {entry['round']}: {entry['reason']} ({entry['n_clusters']} clusters)")
    
    print("\n✅ All clustering tests passed!")
