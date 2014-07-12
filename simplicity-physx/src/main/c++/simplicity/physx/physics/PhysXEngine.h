/*
* Copyright © 2014 Simple Entertainment Limited
*
* This file is part of The Simplicity Engine.
*
* The Simplicity Engine is free software: you can redistribute it and/or modify it under the terms of the GNU General
* Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option)
* any later version.
*
* The Simplicity Engine is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
* implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
* details.
*
* You should have received a copy of the GNU General Public License along with The Simplicity Engine. If not, see
* <http://www.gnu.org/licenses/>.
*/
#ifndef PHYSXENGINE_H_
#define PHYSXENGINE_H_

#include <simplicity/engine/Engine.h>
#include <simplicity/math/Vector.h>

#include "PhysXBody.h"

namespace simplicity
{
	namespace simphysx
	{
		class PhysXEngine : public Engine
		{
			public:
				PhysXEngine(const Vector3& gravity, float fixedTimeStep = 0.0f);

				void advance() override;

				physx::PxCooking* getCooking();

				physx::PxPhysics* getPhysics();

				void onAddEntity(Entity& entity) override;

				void onPlay() override;

				void onRemoveEntity(Entity& entity) override;

				void onStop() override;

				void setSimulationEventCallback(std::unique_ptr<physx::PxSimulationEventCallback> simulationEventCallback);

				void setSimulationFilterShader(physx::PxSimulationFilterShader simulationFilterShader);

			private:
				physx::PxDefaultAllocator allocator;

				physx::PxCooking* cooking;

				physx::PxDefaultCpuDispatcher* cpuDispatcher;

				physx::PxDefaultErrorCallback errorCallback;

				float fixedTimeStep;

				physx::PxFoundation* foundation;

				Vector3 gravity;

				physx::PxPhysics* physics;

				physx::PxScene* scene;

				std::unique_ptr<physx::PxSimulationEventCallback> simulationEventCallback;

				physx::PxSimulationFilterShader simulationFilterShader;
		};
	}
}

#endif /* PHYSXENGINE_H_ */
