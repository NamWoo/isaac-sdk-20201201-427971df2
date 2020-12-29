#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xc8e6b84a0a19c835;

using import "math.capnp".Pose3dProto;

# A message to request creation and destruction of actors in simulator
struct ActorGroupProto {
  # A request to spawn a new actor
  struct SpawnRequest {
    # The name given to the spawned actor. This name does not need to be unique. If multiple spawned
    # actors have the same name, the subsequent requests apply to all of them.
    name @0: Text;
    # The object template the spawned actor is instantiated from (e.g. a prefab name in Unity)
    prefab @1: Text;
    # The pose to set the spawned actor (in the simulator's reference frame)
    pose @2: Pose3dProto;
  }
  # List of spawn requests
  spawnRequests @0: List(SpawnRequest);

  # List of names of actors to destroy.
  destroyRequests @1: List(Text);
}

# A message as response to ActorGroupProto, listing current actors in the scene.
struct ActorGroupStatusProto {
  # Currently alive actors
  actors @0: List(Text);
  # Actors that are spawned in last tick (optional)
  spawnedActors @1: List(Text);
  # Actors that are destroyed in last tick (optional)
  destroyedActors @2: List(Text);
}