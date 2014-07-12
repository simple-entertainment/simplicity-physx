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
#include <simplicity/Simplicity.h>

#include "../math/PhysXMatrix.h"
#include "../math/PhysXVector.h"
#include "PhysXEngine.h"

using namespace physx;
#if defined(DEBUG) || defined(_DEBUG)
using namespace physx::debugger::comm;
#endif
using namespace std;

namespace simplicity
{
	namespace simphysx
	{
		PhysXEngine::PhysXEngine(const Vector3& gravity, float fixedTimeStep) :
			allocator(),
			cooking(),
			cpuDispatcher(nullptr),
			errorCallback(),
			fixedTimeStep(fixedTimeStep),
			foundation(nullptr),
			gravity(gravity),
			physics(nullptr),
			scene(nullptr),
			simulationEventCallback(),
			simulationFilterShader(PxDefaultSimulationFilterShader)
		{
			foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback);

			PxTolerancesScale tolerancesScale;

#if defined(DEBUG) || defined(_DEBUG)
			PxProfileZoneManager& profileZoneManager =
				PxProfileZoneManager::createProfileZoneManager(foundation);
			physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, tolerancesScale, true, &profileZoneManager);

			if (physics->getPvdConnectionManager() != nullptr)
			{
				PxInitExtensions(*physics);
				PvdConnection* debuggerConnection = PxVisualDebuggerExt::createConnection(physics->getPvdConnectionManager(),
					"localhost", 5425, 100, PxVisualDebuggerExt::getAllConnectionFlags());
				if (debuggerConnection != nullptr)
				{
					debuggerConnection->release();
				}
			}
#else
			physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, tolerancesScale);
#endif

			cooking = PxCreateCooking(PX_PHYSICS_VERSION, *foundation, PxCookingParams(tolerancesScale));
		}

		void PhysXEngine::advance()
		{
			if (fixedTimeStep == 0.0f)
			{
				scene->simulate(Simplicity::getDeltaTime());
			}
			else
			{
				scene->simulate(fixedTimeStep);
			}

			scene->fetchResults(true);

			PxU32 activeTransformCount;
			const PxActiveTransform* activeTransforms = scene->getActiveTransforms(activeTransformCount);

			for (PxU32 index = 0; index < activeTransformCount; index++)
			{
				Entity* entity = static_cast<Entity*>(activeTransforms[index].actor->userData);

				Matrix44 newTransform = PhysXMatrix::toMatrix44(activeTransforms[index].actor2World);
				if (newTransform != entity->getTransform())
				{
					entity->setTransform(newTransform);
					Simplicity::getScene()->updateGraphs(*entity);
				}
			}
		}

		PxCooking* PhysXEngine::getCooking()
		{
			return cooking;
		}

		PxPhysics* PhysXEngine::getPhysics()
		{
			return physics;
		}

		void PhysXEngine::onAddEntity(Entity& entity)
		{
			vector<PhysXBody*> entityBodies = entity.getComponents<PhysXBody>();
			for (unsigned int index = 0; index < entityBodies.size(); index++)
			{
				entityBodies[index]->getActor()->userData = &entity;
				scene->addActor(*entityBodies[index]->getActor());
			}
		}

		void PhysXEngine::onPlay()
		{
			PxSceneDesc sceneDesc(physics->getTolerancesScale());
			cpuDispatcher = PxDefaultCpuDispatcherCreate(1);
			sceneDesc.cpuDispatcher = cpuDispatcher;
			sceneDesc.flags = PxSceneFlag::eENABLE_ACTIVETRANSFORMS;
			sceneDesc.gravity = PhysXVector::toPxVec3(gravity);
			sceneDesc.filterShader = simulationFilterShader;
			sceneDesc.simulationEventCallback = simulationEventCallback.get();

			scene = physics->createScene(sceneDesc);
		}

		void PhysXEngine::onRemoveEntity(Entity& entity)
		{
			vector<PhysXBody*> entityBodies = entity.getComponents<PhysXBody>();
			for (unsigned int index = 0; index < entityBodies.size(); index++)
			{
				scene->removeActor(*entityBodies[index]->getActor());
			}
		}

		void PhysXEngine::onStop()
		{
			if (physics != nullptr)
			{
				physics->release();
			}

			if (foundation != nullptr)
			{
				foundation->release();
			}
		}

		void PhysXEngine::setSimulationEventCallback(unique_ptr<PxSimulationEventCallback> simulationEventCallback)
		{
			this->simulationEventCallback = move(simulationEventCallback);
		}

		void PhysXEngine::setSimulationFilterShader(PxSimulationFilterShader simulationFilterShader)
		{
			this->simulationFilterShader = simulationFilterShader;
		}
	}
}
