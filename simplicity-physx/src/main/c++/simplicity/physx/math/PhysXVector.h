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
#ifndef PHYSXVECTOR_H_
#define PHYSXVECTOR_H_

#include <PxPhysicsAPI.h>

#include <simplicity/math/Vector.h>

namespace simplicity
{
	namespace physx
	{
		namespace PhysXVector
		{
			::physx::PxVec3 toPxVec3(const Vector3& original);

			Vector3 toVector3(const ::physx::PxVec3& original);
		}
	}
}

#endif /* PHYSXVECTOR_H_ */
