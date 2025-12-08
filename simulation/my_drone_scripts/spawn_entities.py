#!/usr/bin/env python3
"""
Entity Spawning Script for AirSim
Spawns humans and animals throughout the AirSimNH environment
Falls back to colored marker spheres if assets are unavailable
"""

import airsim
import numpy as np
import time
import sys

class EntitySpawner:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        print("✅ Connected to AirSim")
        
        self.spawned_entities = []
        
        # Define entity types with fallback colors
        self.entity_types = {
            'person': {'color': [1.0, 0.8, 0.6], 'scale': 1.0, 'label': '🧍 Person'},
            'dog': {'color': [0.6, 0.4, 0.2], 'scale': 0.5, 'label': '🐕 Dog'},
            'cat': {'color': [1.0, 0.6, 0.0], 'scale': 0.3, 'label': '🐈 Cat'},
            'horse': {'color': [0.4, 0.3, 0.2], 'scale': 1.5, 'label': '🐴 Horse'},
            'cow': {'color': [0.9, 0.9, 0.9], 'scale': 1.2, 'label': '🐄 Cow'},
            'sheep': {'color': [0.95, 0.95, 0.8], 'scale': 0.7, 'label': '🐑 Sheep'},
            'bird': {'color': [0.2, 0.5, 0.8], 'scale': 0.2, 'label': '🐦 Bird'},
            'elephant': {'color': [0.5, 0.5, 0.5], 'scale': 2.0, 'label': '🐘 Elephant'},
        }
        
    def check_available_assets(self):
        """Check what assets are available in the AirSim environment"""
        try:
            assets = self.client.simListAssets()
            print(f"📦 Available assets in environment: {len(assets)}")
            
            # Look for person/animal related assets
            animal_assets = [a for a in assets if any(keyword in a.lower() 
                           for keyword in ['person', 'human', 'man', 'woman', 'dog', 'cat', 
                                          'horse', 'cow', 'animal', 'bird', 'sheep'])]
            
            if animal_assets:
                print(f"🎯 Found {len(animal_assets)} potential entity assets:")
                for asset in animal_assets[:10]:  # Show first 10
                    print(f"   - {asset}")
                return animal_assets
            else:
                print("⚠️  No person/animal assets found. Will use colored markers.")
                return []
        except Exception as e:
            print(f"⚠️  Could not list assets: {e}. Using fallback markers.")
            return []
    
    def spawn_marker_entity(self, entity_type, position, entity_id):
        """
        Fallback: Create a colored sphere marker to represent an entity
        Uses simSpawnObject with basic shapes if available
        """
        try:
            entity_info = self.entity_types.get(entity_type, self.entity_types['person'])
            
            # Try to spawn a sphere/cube as marker
            pose = airsim.Pose(
                position_val=airsim.Vector3r(position[0], position[1], position[2])
            )
            scale = airsim.Vector3r(
                entity_info['scale'], 
                entity_info['scale'], 
                entity_info['scale']
            )
            
            obj_name = f"{entity_type}_{entity_id}"
            
            # Try spawning with different shape names
            for shape in ['Sphere', 'SM_Sphere', 'Cube', 'SM_Cube', 'Cylinder']:
                try:
                    result = self.client.simSpawnObject(
                        object_name=obj_name,
                        asset_name=shape,
                        pose=pose,
                        scale=scale,
                        physics_enabled=False
                    )
                    
                    if result:
                        print(f"   ✅ Spawned {entity_info['label']} ({shape}) at ({position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f})")
                        self.spawned_entities.append(obj_name)
                        return True
                except Exception as e:
                    if 'Cylinder' in shape:  # Log last attempt
                        print(f"   ⚠️  All shape attempts failed for {obj_name}: {str(e)[:50]}")
                    continue
            
            # If all spawn attempts fail, just record the position for YOLO to detect in other ways
            print(f"   ⚠️  Could not spawn mesh for {entity_info['label']}, marking position only")
            return False
            
        except Exception as e:
            print(f"   ❌ Error spawning marker: {e}")
            return False
    
    def spawn_entity(self, entity_type, position, entity_id, available_assets):
        """
        Spawn an entity (person/animal) at the specified position
        Tries real assets first, falls back to markers
        """
        try:
            # First try: Use actual assets if available
            matching_assets = [a for a in available_assets if entity_type.lower() in a.lower()]
            
            if matching_assets:
                asset_name = matching_assets[0]
                entity_info = self.entity_types.get(entity_type, self.entity_types['person'])
                
                pose = airsim.Pose(
                    position_val=airsim.Vector3r(position[0], position[1], position[2])
                )
                scale = airsim.Vector3r(entity_info['scale'], entity_info['scale'], entity_info['scale'])
                
                obj_name = f"{entity_type}_{entity_id}"
                
                result = self.client.simSpawnObject(
                    object_name=obj_name,
                    asset_name=asset_name,
                    pose=pose,
                    scale=scale,
                    physics_enabled=False
                )
                
                if result:
                    print(f"   ✅ Spawned {entity_info['label']} (asset) at ({position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f})")
                    self.spawned_entities.append(obj_name)
                    return True
            
            # Fallback: Use marker
            return self.spawn_marker_entity(entity_type, position, entity_id)
            
        except Exception as e:
            print(f"   ⚠️  Asset spawn failed, trying marker: {e}")
            return self.spawn_marker_entity(entity_type, position, entity_id)
    
    def create_entity_distribution(self):
        """
        Create a realistic distribution of people and animals across the map
        Spreads entities in interesting locations for drone mapping
        """
        entities = []
        entity_id = 0
        
        # VISIBLE TEST ENTITIES - To the SIDE of drones, not blocking path
        print("\n🎯 Spawning test entities (positioned to avoid blocking flight path)...")
        test_positions = [
            (5, 15, -2),   # To the side of Drone0
            (10, 20, -2),  # Further to the side
            (15, -15, -2), # Other side
        ]
        test_types = ['person', 'dog', 'cat']
        
        for i, (x, y, z) in enumerate(test_positions):
            entities.append({
                'type': test_types[i],
                'position': [x, y, z],
                'id': entity_id
            })
            entity_id += 1
        
        # Grid pattern with some randomness - spread out more
        grid_size = 60  # meters between entities (increased for spacing)
        offset_range = 10  # random offset
        
        # Create grid of people (most common) - avoid center area (0,0)
        for x in range(-100, 100, grid_size):
            for y in range(-100, 100, grid_size):
                # Skip the center area where drones spawn
                if abs(x) < 30 and abs(y) < 30:
                    continue
                
                # Random offset for natural placement
                x_pos = x + np.random.uniform(-offset_range, offset_range)
                y_pos = y + np.random.uniform(-offset_range, offset_range)
                z_pos = -2.0  # Ground level (NED coordinates, negative is up)
                
                # 50% chance of person, 50% chance of animal
                if np.random.random() < 0.5:
                    entity_type = 'person'
                else:
                    # Random animal
                    entity_type = np.random.choice(['dog', 'cat', 'horse', 'cow', 'sheep', 'bird'])
                
                entities.append({
                    'type': entity_type,
                    'position': [x_pos, y_pos, z_pos],
                    'id': entity_id
                })
                entity_id += 1
        
        # Add some concentrated groups (like a park or farm)
        # Group 1: "Park" with people and dogs
        for i in range(10):
            x_pos = 50 + np.random.uniform(-15, 15)
            y_pos = 50 + np.random.uniform(-15, 15)
            entity_type = 'person' if i < 7 else 'dog'
            entities.append({
                'type': entity_type,
                'position': [x_pos, y_pos, -2.0],
                'id': entity_id
            })
            entity_id += 1
        
        # Group 2: "Farm" with animals
        for i in range(15):
            x_pos = -80 + np.random.uniform(-20, 20)
            y_pos = -80 + np.random.uniform(-20, 20)
            entity_type = np.random.choice(['cow', 'horse', 'sheep'])
            entities.append({
                'type': entity_type,
                'position': [x_pos, y_pos, -2.0],
                'id': entity_id
            })
            entity_id += 1
        
        print(f"📍 Planned {len(entities)} entities for spawning")
        return entities
    
    def spawn_all_entities(self):
        """Main function to spawn all entities"""
        print("\n" + "="*70)
        print("  🌍 ENTITY SPAWNING FOR 3D MAPPING DEMO")
        print("="*70 + "\n")
        
        # Check available assets
        available_assets = self.check_available_assets()
        print()
        
        # Create entity distribution
        entities = self.create_entity_distribution()
        
        # Spawn entities
        print(f"\n🚀 Spawning {len(entities)} entities...")
        print("-" * 70)
        
        success_count = 0
        for entity in entities:
            if self.spawn_entity(
                entity['type'], 
                entity['position'], 
                entity['id'],
                available_assets
            ):
                success_count += 1
            time.sleep(0.05)  # Small delay to avoid overwhelming the API
        
        print("-" * 70)
        print(f"\n✅ Successfully spawned {success_count}/{len(entities)} entities")
        print(f"📝 Entity names stored for cleanup: {len(self.spawned_entities)}")
        
        # Save entity info for YOLO detection reference
        self.save_entity_manifest(entities)
        
        return self.spawned_entities
    
    def save_entity_manifest(self, entities):
        """Save entity positions to file for detection reference"""
        try:
            with open('/tmp/airsim_entity_manifest.txt', 'w') as f:
                f.write("# AirSim Entity Manifest\n")
                f.write("# Format: type,x,y,z,id\n\n")
                for entity in entities:
                    pos = entity['position']
                    f.write(f"{entity['type']},{pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f},{entity['id']}\n")
            print(f"💾 Entity manifest saved to /tmp/airsim_entity_manifest.txt")
        except Exception as e:
            print(f"⚠️  Could not save manifest: {e}")
    
    def cleanup(self):
        """Remove all spawned entities"""
        print("\n🧹 Cleaning up spawned entities...")
        for obj_name in self.spawned_entities:
            try:
                self.client.simDestroyObject(obj_name)
                print(f"   ✅ Removed {obj_name}")
            except Exception as e:
                print(f"   ⚠️  Could not remove {obj_name}: {e}")
        print("✅ Cleanup complete")

def main():
    spawner = EntitySpawner()
    
    try:
        spawner.spawn_all_entities()
        
        print("\n" + "="*70)
        print("  ✅ ENTITY SPAWNING COMPLETE")
        print("="*70)
        print("\nEntities are now in the environment for drone detection.")
        print("Press Ctrl+C to cleanup and exit...")
        
        # Keep script running
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\n🛑 Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        spawner.cleanup()

if __name__ == "__main__":
    main()
