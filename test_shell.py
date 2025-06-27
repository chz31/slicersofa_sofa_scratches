import Sofa
import SofaRuntime
import Sofa.Gui
from Sofa.constants import Key
from scipy.spatial.transform import Rotation
import numpy as np


# import PneuNetController

def main():
    # Make sure to load all SOFA libraries
    SofaRuntime.importPlugin("SofaBaseMechanics")
    SofaRuntime.importPlugin("SofaOpenglVisual")
    SofaRuntime.importPlugin("CImgPlugin")
    # SofaRuntime.importPlugin("Sofa.Component.Shell")

    # Call the above function to create the scene graph
    # root.name="root"
    root = Sofa.Core.Node("root")  # root node creation (previously provided by runSofa)
    createScene(root)  # scene graph initialization (p.provided by runSofa)

    # Once defined, initialization of the scene graph
    Sofa.Simulation.init(root)

    # Find out the supported GUIs
    print("Supported GUIs are: " + Sofa.Gui.GUIManager.ListSupportedGUI(","))
    # Launch the GUI (qt or qglviwere)
    Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    # Initialization of the scene will be done here
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()
    print("GUI was closed")

    # Run the simulation for 10 steps
    # print("time is = " + str(root.time.value))
    # for iteration in range(10):
    #     print(f'Iteration ${iteration}')
    #     Sofa.Simulation.animate(root, root.dt.value)

    print("Simulation made 10 time steps. Done.")


def createScene(root):
    root.name = "root"
    root.gravity = [0, 0, 0]
    root.dt = 0.01
    # root.addObject('DefaultAnimationLoop', computeBoundingBox=False)

    confignode = root.addChild("Config")
    confignode.addObject('RequiredPlugin', name='Sofa.Component.Visual')  # Needed to use components [VisualGrid]
    confignode.addObject('RequiredPlugin', name="Sofa.Component.AnimationLoop", printLog=False)
    # For CGLlinearSolver
    confignode.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Iterative")
    #
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.LinearSolver.Direct')  # Needed to use components [SparseLDLSolver]
    confignode.addObject('RequiredPlugin', name="Sofa.Component.ODESolver.Backward")
    confignode.addObject('RequiredPlugin', name="Sofa.Component.MechanicalLoad")
    confignode.addObject('RequiredPlugin', name="Sofa.Component.IO.Mesh")
    confignode.addObject('RequiredPlugin', name="Sofa.GL.Component.Rendering3D",
                         printLog=False)  # Needed to use components [OglModel]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Mapping.NonLinear')  # Needed to use components [RigidMapping]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.StateContainer')  # Needed to use components [MechanicalObject]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Constraint.Lagrangian.Correction')  # Needed to use components [UncoupledConstraintCorrection]
    confignode.addObject('RequiredPlugin', name='Sofa.Component.Mass')  # Needed to use components [UniformMass]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Collision.Geometry')  # Needed to use components [LineCollisionModel,PointCollisionModel,TriangleCollisionModel]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Topology.Container.Constant')  # Needed to use components [MeshTopology]

    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Collision.Detection.Algorithm')  # Needed to use components [BVHNarrowPhase,BruteForceBroadPhase,CollisionPipeline]                                                                       onPipeline]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Collision.Detection.Intersection')  # Needed to use components [LocalMinDistance, DiscreteIntersection]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Collision.Response.Contact')  # Needed to use components [RuleBasedContactManager, CollisionResponse]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Constraint.Lagrangian.Solver')  # Needed to use components [GenericConstraintSolver]

    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Topology.Container.Dynamic')  # Needed to use components [TetrahedronSetGeometryAlgorithms TetrahedronSetTopologyContainer
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.SolidMechanics.FEM.Elastic')  # Needed to use components [TetrahedronFEMForceField]
    confignode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Mapping.Linear')  # Needed to use components [IdentityMapping BarycentricMapping]
    confignode.addObject('RequiredPlugin',
                         name='MultiThreading')  # Needed to use components [IdentityMapping BarycentricMapping]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Constraint.Lagrangian.Model')  # Needed to use components [IdentityMapping BarycentricMapping]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Engine.Select')  # Needed to use components [IdentityMapping BarycentricMapping]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.SolidMechanics.Spring')  # Needed to use components [IdentityMapping BarycentricMapping]
    confignode.addObject('RequiredPlugin',
                         name='Shell')
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Constraint.Projective')  # Needed to use components [FixedProjectiveConstraint]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.GUI.Component')  # Needed to use components [AttachBodyButtonSetting]



    root.addObject('FreeMotionAnimationLoop', parallelCollisionDetectionAndFreeMotion=True)
    root.addObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="1000")
    confignode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    root.addObject("VisualGrid", nbSubdiv=10, size=1000)

    # Collision pipeline
    root.addObject('CollisionPipeline')
    root.addObject('ParallelBruteForceBroadPhase')
    root.addObject('ParallelBVHNarrowPhase')
    root.addObject('CollisionResponse', responseParams="mu=" + str(5.0), name='Response',
                   response='FrictionContactConstraint')  # FrictionContactConstraint or PenaltyContact
    root.addObject('LocalMinDistance', alarmDistance=2, contactDistance=0.2)

    # root.addObject('CollisionResponse', response='FrictionContact', responseParams='mu=0.9')

    # add a mechanical model, so that all our future elements will have the same total mass, volume and inertia matrix
    totalMass = 1.0
    volume = 1.0
    inertiaMatrix = [1., 0., 0., 0., 1., 0., 0., 0., 1.]

    # mesh loader for msh

    # mesh loader for msh
    # plateCenterOfMass = [-45.29316542124225, -85.27826548370909, -19.06820484608216, 0, 0, 0, 1]
    # plateKeyControl = root.addChild('plateKeyControl')
    # plateCMState = plateKeyControl.addObject('MechanicalObject', name='controllerMO', position=plateCenterOfMass,
    #                                          template='Rigid3')
    #
    # root.addObject(plateController(name="plateController", mstate=plateCMState, delta=0.05))

    # plateMeshLoaderCoarse = root.addObject('MeshGmshLoader', name='plateMeshLoaderCoarse', filename='/home/chi/Documents/orbit_SOFA/soft_tissue-models/right_aligned_plate_sofa.msh', scale=1.0)

    plateCenterOfMass = [-33.2873, -89.1452, -14.29893, 0, 0, 0, 1]

    plateMeshLoaderFine = root.addObject('MeshOBJLoader', name='plateMeshLoaderFine',
                                         filename='/home/chi/Documents/test_sofa_slicersofa_models/plateSurfaceModel.obj',
                                         scale=1.0)

    # Creating the falling sphere object
    root.addObject('AttachBodyButtonSetting', stiffness=0.1)
    # root.addObject('DefaultAnimationLoop')

    plate2dNode = root.addChild('plate2dNode')
    plate2dNode.addObject('EulerImplicitSolver')
    plate2dNode.addObject('SparseLDLSolver')
    # plate2dNode.addObject('MeshOBJLoader', filename='mesh/plate2dNode1.obj')
    plate2dNode.addObject('MeshTopology', src=plateMeshLoaderFine.getLinkPath())
    plate2dNode.addObject('MechanicalObject', template='Rigid3', position=plateCenterOfMass)
    plate2dNode.addObject('UniformMass', totalMass=0.005)
    plate2dNode.addObject('BoxROI', box=[0, 0, 0, -80, -80, -50], drawBoxes=True)
    plate2dNode.addObject('FixedProjectiveConstraint', indices=plate2dNode.BoxROI.indices.getLinkPath())
    plate2dNode.addObject('TriangularShellForceField', youngModulus=5e4,
                     poissonRatio=0.45, thickness=0.1)
    plate2dNode.addObject('LinearSolverConstraintCorrection')


    # Plate Collision model
    plateCollision = plate2dNode.addChild('plateCollision')
    plateCollision.addObject('MeshTopology', src=plateMeshLoaderFine.getLinkPath())
    plateCollision.addObject('MechanicalObject', name='CollisionMO')
    plateCollision.addObject('TriangleCollisionModel', name='CollisionModel', contactStiffness='100')
    plateCollision.addObject('LineCollisionModel')
    # plateCollision.addObject('PointCollisionModel')
    plateCollision.addObject('RigidMapping', globalToLocalCoords=True)


    visu = plate2dNode.addChild('Visu')
    visu.addObject('OglModel', src=plate2dNode.MeshTopology.getLinkPath())
    visu.addObject('IdentityMapping')

    return root


# if we are in a python environment
if __name__ == '__main__':
    main()


# Class ManageParticles(Sofa.Core.Controller):
# Can create keyboard short cut with customized ForceFiel
# Add and remove particles by creating functions


