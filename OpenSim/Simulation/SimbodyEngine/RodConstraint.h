#ifndef OPENSIM_ROD_CONSTRAINT_H_
#define OPENSIM_ROD_CONSTRAINT_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  RodConstraint.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2023 Stanford University and the Authors                *
 * Author(s): Jennifer Maier                                                  *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

// INCLUDE
#include "Constraint.h"

namespace OpenSim {

class PhysicalFrame;

//=============================================================================
//=============================================================================
/**
 * A class implementing a Rod Constraint. This constraint consists of one
 * constraint equation that enforces a constant distance between a point on
 * one body and a point on another body.
 *
 * The underlying SimTK::Constraint in Simbody is a SimTK::Constraint::Rod.
 *
 * @author Jennifer Maier
 */
class OSIMSIMULATION_API RodConstraint : public Constraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(RodConstraint, Constraint);

public:
    OpenSim_DECLARE_PROPERTY(location_body_1, SimTK::Vec3,
            "Location of the point in first body specified in body_1 reference "
            "frame.");
    OpenSim_DECLARE_PROPERTY(location_body_2, SimTK::Vec3,
            "Location of the point in second body specified in body_2 "
            "reference frame.");
    OpenSim_DECLARE_PROPERTY(rod_length, double,
            "Fixed distance between the specified points in the two bodies");

    OpenSim_DECLARE_SOCKET(body_1, PhysicalFrame,
            "A frame fixed to the first body participating in the constraint.");
    OpenSim_DECLARE_SOCKET(body_2, PhysicalFrame,
            "A frame fixed to the second body participating in the "
            "constraint.");

    //=============================================================================
    // METHODS
    //=============================================================================
public:
    // CONSTRUCTION
    RodConstraint();
    /**
     * Convenience Constructor.
     *
     * @param body1          first PhysicalFrame connected by the constraint
     * @param locationBody1  point fixed on body1 where the constraint is
     * applied
     * @param body2          second PhysicalFrame connected by the constraint
     * @param locationBody2: point fixed on body2 where the constraint is
     * applied
     * @param rodLength:     fixed distance between the two points
     */
    RodConstraint(const PhysicalFrame& body1,
            const SimTK::Vec3& locationBody1, const PhysicalFrame& body2,
            const SimTK::Vec3& locationBody2, double rodLength);
    virtual ~RodConstraint();

    // SET
    void setBody1ByName(const std::string& aBodyName);
    void setBody1PointLocation(SimTK::Vec3 location);
    void setBody2ByName(const std::string& aBodyName);
    void setBody2PointLocation(SimTK::Vec3 location);
    void setRodLength(double);

protected:
    /**
     * Extend Component Interface.
     */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    /** Updating XML formating to latest revision */
    void updateFromXMLNode(
            SimTK::Xml::Element& aNode, int versionNumber) override;

private:
    /** Construct RodConstraint's properties */
    void constructProperties();

    void setNull();

//=============================================================================
}; // END of class RodConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_ROD_CONSTRAINT_H_
