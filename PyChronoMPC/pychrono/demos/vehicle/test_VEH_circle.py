# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban
# =============================================================================
#
# Demonstration of simulating two vehicles simultaneously.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

import MPC

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")
    MPC.SayHello()

    step_size = 0.005

    sys = chrono.ChSystemNSC()
    sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sys.SetSolverMaxIterations(150)
    sys.SetMaxPenetrationRecoverySpeed(4.0)

    # SIMPLE TERRAIN
    '''
    # Create the terrain
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()
    '''
    # COMPLEX TERRAIN
    # Create the terrain with multiple patches
    terrain = veh.RigidTerrain(sys)

    patch1_mat = chrono.ChMaterialSurfaceNSC()
    patch1_mat.SetFriction(0.9)
    patch1_mat.SetRestitution(0.01)
    patch1 = terrain.AddPatch(patch1_mat, chrono.ChCoordsysD(chrono.ChVectorD(-16, 0, 0), chrono.QUNIT), 32, 20)
    patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)

    patch2_mat = chrono.ChMaterialSurfaceNSC()
    patch2_mat.SetFriction(0.9)
    patch2_mat.SetRestitution(0.01)
    patch2 = terrain.AddPatch(patch2_mat, chrono.ChCoordsysD(chrono.ChVectorD(16, 0, 0.15), chrono.QUNIT), 32, 30);
    patch2.SetColor(chrono.ChColor(1.0, 0.5, 0.5))
    patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)

    patch3_mat = chrono.ChMaterialSurfaceNSC()
    patch3_mat.SetFriction(0.9)
    patch3_mat.SetRestitution(0.01)
    patch3 = terrain.AddPatch(patch3_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, -42, 0), chrono.QUNIT),
                              veh.GetDataFile("terrain/meshes/bump.obj"))
    patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
    patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

    patch4_mat = chrono.ChMaterialSurfaceNSC()
    patch4_mat.SetFriction(0.9)
    patch4_mat.SetRestitution(0.01)
    patch4 = terrain.AddPatch(patch4_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 42, 0), chrono.QUNIT),
                              veh.GetDataFile("terrain/height_maps/bump64.bmp"), 64.0, 64.0, 0.0, 3.0)
    patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6.0, 6.0)

    terrain.Initialize()

    # Create and initialize the first vehicle
    hmmwv_1 = veh.HMMWV_Reduced(sys)
    hmmwv_1.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, -1.5, 1.0), chrono.ChQuaternionD(1, 0, 0, 0)))
    hmmwv_1.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv_1.SetTransmissionType(veh.TransmissionModelType_SIMPLE_MAP)
    hmmwv_1.SetDriveType(veh.DrivelineTypeWV_RWD)
    hmmwv_1.SetTireType(veh.TireModelType_RIGID)
    hmmwv_1.Initialize()
    hmmwv_1.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv_1.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv_1.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv_1.SetWheelVisualizationType(veh.VisualizationType_NONE)
    hmmwv_1.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    # TODO: In our test, we want to get these from MPC
    # driver_data_1 = veh.vector_Entry([veh.DataDriverEntry(time, steering, throttle, breaking, clutch)
    # TODO: theta = (Vr - Vl)/distance_between_wheels
    '''
    driver_data_1 = veh.vector_Entry([veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0, 0.0),
                                      veh.DataDriverEntry(0.5, 0.0, 0.0, 0.0, 0.0),
                                      veh.DataDriverEntry(0.7, 0.3, 0.7, 0.0, 0.0),
                                      veh.DataDriverEntry(1.0, 0.3, 0.7, 0.0, 0.0),
                                      veh.DataDriverEntry(3.0, 0.5, 0.1, 0.0, 0.0)
                                     ])
    '''

    '''
    Driving Tests
    '''
    # Drive Straight Off Edge
    driver_data_1 = veh.vector_Entry([veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0, 0.0),
                                      veh.DataDriverEntry(5, 0.0, 1.0, 0.0, 0.0),  # drive straight
                                      ])

    # Drive Straight and Break
    driver_data_1 = veh.vector_Entry([veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0, 0.0),
                                      veh.DataDriverEntry(5, 0.0, 1.0, 0.0, 0.0),  # drive straight
                                      veh.DataDriverEntry(5, 0.0, 0.0, 2.0, 0.0),  # break
                                      ])

    driver_1 = veh.ChDataDriver(hmmwv_1.GetVehicle(), driver_data_1)
    driver_1.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Two Car Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.SetChaseCameraState(veh.ChChaseCamera.Track)
    vis.SetChaseCameraPosition(chrono.ChVectorD(-10, 0, 2.0))
    vis.Initialize()
    vis.AddLightDirectional()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv_1.GetVehicle())


    # Simulation loop
    hmmwv_1.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = hmmwv_1.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs_1 = driver_1.GetInputs()

        # Update modules (process inputs from other modules)
        driver_1.Synchronize(time)
        hmmwv_1.Synchronize(time, driver_inputs_1, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs_1)

        # Advance simulation for one timestep for all modules
        driver_1.Advance(step_size)
        hmmwv_1.Advance(step_size)
        terrain.Advance(step_size)
        vis.Advance(step_size)

        # Advance state of entire system (containing both vehicles)
        sys.DoStepDynamics(step_size)

    return 0

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with:
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

main()