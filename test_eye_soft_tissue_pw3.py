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
    # confignode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')
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
    meshLoaderCoarse = root.addObject('MeshGmshLoader', name='meshLoaderCoarse',
                                      filename='/home/chi/Documents/orbit_SOFA/soft_tissue-models/orbit_soft_tissue_down.msh',
                                      scale=1.0)
    # meshLoaderCoarse = root.addObject('MeshGmshLoader', name='meshLoaderCoarse', filename='C:/Users/chi.zhang/Desktop/orbit_SOFA/soft_tissue-models/orbit_soft_vol.vtk', scale=1.0)
    meshLoaderFine = root.addObject('MeshOBJLoader', name='meshLoaderFine',
                                    filename='/home/chi/Documents/orbit_SOFA/soft_tissue-models/orbit_soft_tissue_down.obj',
                                    scale=1.0)

    # Creating the falling sphere object
    eyeNode = root.addChild("eye")
    # eyeNode.addObject('MechanicalObject', name="mstate", template="Rigid3", translation2=[0., 0., 0.], rotation2=[0., 0., 0.], showObjectScale=50)

    # Creating the falling eyeNode object
    eyeNode.addObject('EulerImplicitSolver', name='odesolver')
    # eyeNode.addObject('CGLinearSolver', name='Solver', iterations=25, tolerance=1e-05, threshold=1e-05)
    eyeNode.addObject('SparseLDLSolver', name='Solver')
    # eyeNode.addObject('GenericConstraintSolver', name='Solver')

    # tetrahedron topology
    eyeNode.addObject('TetrahedronSetTopologyContainer', name='topo', src=meshLoaderCoarse.getLinkPath())
    eyeNode.addObject('TetrahedronSetGeometryAlgorithms', template='Vec3d', name='GeomAlgo')

    eyeNode.addObject('MechanicalObject', template='Vec3d', name='MechanicalModel', showObject=True)

    eyeNode.addObject('BoxROI', name='FixedPoints', template='Rigid3', position="@MechanicalModel.position",
                      box="0 0 0  -60  -15 100")
    eyeNode.addObject('RestShapeSpringsForceField', name='RestShapeSpringForceField', stiffness=1e6,
                      points="@FixedPoints.indices")

    eyeNode.addObject('TetrahedronFEMForceField', name='FEM', youngModulus=1e4, poissonRatio=0.47, method='large')
    eyeNode.addObject('LinearSolverConstraintCorrection')  # need for gravity filed #LinearSolverConstraint
    # eyeNode.addObject('UncoupledConstraintCorrection', compliance=1e-9)
    eyeNode.addObject('MeshMatrixMass', massDensity=1, topology='@topo')

    collision = eyeNode.addChild('Collision')
    collision.addObject('MeshTopology', src=meshLoaderFine.getLinkPath())
    collision.addObject('MechanicalObject', name='CollisionMO')
    collision.addObject('TriangleCollisionModel', name='CollisionModel')
    collision.addObject('LineCollisionModel')
    collision.addObject('BarycentricMapping', name='CollisionMapping', input='@../MechanicalModel',
                        output='@CollisionMO')

    # Fine visual model for plate with tetrahedron
    fineVisualModel = eyeNode.addChild('FineVisualModel')
    fineVisualModel.addObject('OglModel', name='VisualModel', src=meshLoaderFine.getLinkPath())
    fineVisualModel.addObject('BarycentricMapping', name='Mapping', input='@../MechanicalModel', output='@VisualModel')

    orbitLoaderCoarse = root.addObject('MeshGmshLoader', name='orbitLoaderCoarse',
                                       filename='/home/chi/Documents/orbit_SOFA/soft_tissue-models/right_orbit_repaired.msh',
                                       scale=1.0)
    orbitLoaderFine = root.addObject('MeshOBJLoader', name='orbitLoaderFine',
                                     filename='/home/chi/Documents/orbit_SOFA/soft_tissue-models/right_orbit_repaired.obj',
                                     scale=1.0)

    # Creating the orbit object
    orbitNode = root.addChild("Orbit")
    # orbitNode.addObject('EulerImplicitSolver', name='odesolver') #need for gravity field
    # orbitNode.addObject('CGLinearSolver', name='Solver', iterations=25, tolerance=1e-05, threshold=1e-05) #need for gravity field
    orbitNode.addObject('MechanicalObject', name="mstate", template="Rigid3", showObject=True)
    orbitNode.addObject('UniformMass', name="mass", vertexMass=[totalMass, volume, inertiaMatrix[:]])
    # orbitNode.addObject('UniformMass', name="mass", totalMass=1)
    # orbitNode.addObject('UncoupledConstraintCorrection') #need for gravity filed

    #### Collision subnode for the orbit
    # orbitCollis = orbitNode.addChild('collision_orbit')
    # orbitCollis.addObject('MeshTopology', src=orbitLoaderFine.getLinkPath())
    # orbitCollis.addObject('MechanicalObject', name='CollisionMO')
    # orbitCollis.addObject('TriangleCollisionModel', moving=False, simulated=False)
    # orbitCollis.addObject('LineCollisionModel', moving=False, simulated=False)
    # orbitCollis.addObject('PointCollisionModel', moving=False, simulated=False)
    # orbitCollis.addObject('RigidMapping')

    # Fine visual model for plate with tetrahedron
    orbitVisu = orbitNode.addChild('FineVisualModel')
    orbitVisu.addObject('OglModel', name='VisualModel', src=orbitLoaderFine.getLinkPath())
    orbitVisu.addObject('RigidMapping')

    # mesh loader for msh
    plateCenterOfMass = [-45.29316542124225, -85.27826548370909, -19.06820484608216, 0, 0, 0, 1]
    plateKeyControl = root.addChild('plateKeyControl')
    plateCMState = plateKeyControl.addObject('MechanicalObject', name='controllerMO', position=plateCenterOfMass,
                                             template='Rigid3')

    root.addObject(plateController(name="plateController", mstate=plateCMState, delta=0.05))

    # plateMeshLoaderCoarse = root.addObject('MeshGmshLoader', name='plateMeshLoaderCoarse', filename='/home/chi/Documents/orbit_SOFA/soft_tissue-models/right_aligned_plate_sofa.msh', scale=1.0)
    plateMeshLoaderFine = root.addObject('MeshOBJLoader', name='plateMeshLoaderFine',
                                         filename='/home/chi/Documents/orbit_SOFA/soft_tissue-models/right_plate_sofa2.obj',
                                         scale=1.0)

    # Creating the falling sphere object
    plateNode = root.addChild("plate")

    # Creating the falling plateNode object
    plateNode.addObject('EulerImplicitSolver', name='odesolver')
    plateNode.addObject('CGLinearSolver', name='Solver', iterations=25, tolerance=1e-05, threshold=1e-05)

    # for aligned plate model
    plateMo = plateNode.addObject('MechanicalObject', template='Rigid3', name='MechanicalModel',
                                  position=plateCenterOfMass, showObject=True, showObjectScale=1.0)
    plateNode.addObject('UniformMass', name="mass", vertexMass=[totalMass, volume, inertiaMatrix[:]])
    # plateNode.addObject('UncoupledConstraintCorrection') #need for gravity filed #LinearSolverConstraint
    plateNode.addObject('BilateralLagrangianConstraint', template='Rigid3',
                        object1=plateMo.linkpath,
                        object2=plateCMState.linkpath,
                        first_point=0,
                        second_point=0)

    plateNode.addObject('UncoupledConstraintCorrection')  # need for gravity filed #LinearSolverConstraint

    # Plate Collision model
    plateCollision = plateNode.addChild('plateCollision')
    plateCollision.addObject('MeshTopology', src=plateMeshLoaderFine.getLinkPath())
    plateCollision.addObject('MechanicalObject', name='CollisionMO')
    plateCollision.addObject('TriangleCollisionModel', name='CollisionModel', contactStiffness='100')
    plateCollision.addObject('LineCollisionModel')
    # plateCollision.addObject('PointCollisionModel')
    plateCollision.addObject('RigidMapping', globalToLocalCoords=True)

    # Fine visual model for plate with tetrahedron
    plateFineVisualModel = plateNode.addChild('FineVisualModel')
    plateFineVisualModel.addObject('OglModel', name='VisualModel', src=plateMeshLoaderFine.getLinkPath())
    # fineVisualModel.addObject('BarycentricMapping', name='Mapping', input='@../MechanicalModel', output='@VisualModel')
    plateFineVisualModel.addObject('RigidMapping', globalToLocalCoords=True)

    # Scoop model with controller
    scoopCenterOfMass = [-29.699957847595215, -65.09541893005371, 9.058785855770111, 0, 0, 0, 1]

    toolController = root.addChild('toolController')
    toolCMState = toolController.addObject('MechanicalObject', name='controllerMO', position=scoopCenterOfMass,
                                           template='Rigid3')

    root.addObject(ScoopController(name="ScoopController", mstate=toolCMState, delta=0.05))

    # Creating the retraction tool object
    toolMeshLoaderFine = root.addObject('MeshOBJLoader', name='toolMeshLoaderFine',
                                        filename='/home/chi/Documents/orbit_SOFA/soft_tissue-models/tool.obj',
                                        scale=1.0)
    toolNode = root.addChild("tool")
    # toolNode.addObject('MechanicalObject', name="mstate", template="Rigid3", translation2=[0., 0., 0.], rotation2=[0., 0., 0.], showObjectScale=50)

    # Creating the falling toolNode object
    toolNode.addObject('EulerImplicitSolver', name='odesolver')
    toolNode.addObject('CGLinearSolver', name='Solver', iterations=25, tolerance=1e-05, threshold=1e-05)

    toolMo = toolNode.addObject('MechanicalObject', template='Rigid3', name='MechanicalModel',
                                position=scoopCenterOfMass, showObject=True, showObjectScale=1.0)
    toolNode.addObject('UniformMass', name="mass", totalMass=10)
    toolNode.addObject('BilateralLagrangianConstraint', template='Rigid3',
                       object1=toolMo.linkpath,
                       object2=toolCMState.linkpath,
                       first_point=0,
                       second_point=0)

    toolNode.addObject('UncoupledConstraintCorrection')  # need for gravity filed #LinearSolverConstraint

    toolCollision = toolNode.addChild('toolCollision')
    toolCollision.addObject('MeshTopology', src=toolMeshLoaderFine.getLinkPath())
    toolCollision.addObject('MechanicalObject', name='CollisionMO')
    toolCollision.addObject('TriangleCollisionModel', name='CollisionModel', contactStiffness='100')
    toolCollision.addObject('LineCollisionModel')
    toolCollision.addObject('RigidMapping', globalToLocalCoords=True)

    # Fine visual model for plate with tetrahedron
    toolFineVisualModel = toolNode.addChild('FineVisualModel')
    toolFineVisualModel.addObject('OglModel', name='VisualModel', src=toolMeshLoaderFine.getLinkPath())
    toolFineVisualModel.addObject('RigidMapping', globalToLocalCoords=True)

    return root


# if we are in a python environment
if __name__ == '__main__':
    main()


# Class ManageParticles(Sofa.Core.Controller):
# Can create keyboard short cut with customized ForceFiel
# Add and remove particles by creating functions





class ScoopController(Sofa.Core.Controller):
    """
    This Controller simply takes the cableActuator's value
    and increases / decreases it depending on the pressed key ('+' or '-')
    """

    def __init__(self, mstate, delta=1.0, *a, **kw):
        """
        In the ctor, we want to first call the constructor for the parent class (trampoline)
        We then store the node we want to retrieve the actuator from in the class
        (Sofa.Core.Base.getContext() could also have been used here instead,
        or a link between aCableActuator and the controller could have been used too)
        """
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.mstate = mstate
        self.delta = delta
        return

    def onKeypressedEvent(self, e):
        """
        Events methods are named after the actual event names (Event::GetClassName() in C++),
        with a prepended "on" prefix. Thus, this Event is the KeypressedEvent class in C++
        The onXXXEvent method takes a dictionary as a parameter, containing the useful
        values stored in the event class, e.g. here, the pressed key
        """

        currentPos = self.mstate.position[0]

        # Move on Y axis
        if e["key"] == Sofa.constants.Key.KP_4:
            currentPos[1] += self.delta
        elif e["key"] == Sofa.constants.Key.KP_6:
            currentPos[1] -= self.delta
        elif e["key"] == Sofa.constants.Key.KP_8:  # Move on Z axis
            currentPos[2] += self.delta
        elif e["key"] == Sofa.constants.Key.KP_2:
            currentPos[2] -= self.delta
        elif e["key"] == Sofa.constants.Key.leftarrow:
            currentPos[3:7] = self.addQuaternion(currentPos[3:7],[self.delta/10, 0, 0])
        elif e["key"] == Sofa.constants.Key.rightarrow:
            currentPos[3:7] = self.addQuaternion(currentPos[3:7],[-self.delta/10, 0, 0])
        elif e["key"] == Sofa.constants.Key.uparrow:
            currentPos[3:7] = self.addQuaternion(currentPos[3:7],[0, self.delta/10, 0])
        elif e["key"] == Sofa.constants.Key.downarrow:
            currentPos[3:7] = self.addQuaternion(currentPos[3:7],[0, -self.delta/10, 0])
        elif e["key"] == Sofa.constants.Key.plus:
            currentPos[3:7] = self.addQuaternion(currentPos[3:7],[0, 0, self.delta/10])
        elif e["key"] == Sofa.constants.Key.minus:
            currentPos[3:7] = self.addQuaternion(currentPos[3:7],[0, 0, -self.delta/10])

        self.mstate.position = [currentPos]

    def addQuaternion(self, quat, delta_array):
        # Example displacement
        # displacement = np.array(delta_array)
        # direction = displacement / np.linalg.norm(displacement)  # Normalize

        # Reference direction (e.g., original facing direction)
        # ref = np.array([0, 1, 0])  # Y-axis

        # Compute rotation that aligns ref -> direction
        # rotation_vector = np.cross(ref, direction)
        # angle = np.arccos(np.dot(ref, direction))
        # rot = Rotation.from_rotvec(rotation_vector / np.linalg.norm(rotation_vector) * angle)
        #
        # quaternion = rot.as_quat()

        baseEuler = Rotation.from_quat(quat).as_euler('xyz')

        baseEuler += delta_array # in radians

        rot = Rotation.from_euler('xyz', baseEuler, degrees=False)

        # Get quaternion [x, y, z, w]
        quat = rot.as_quat()
        return quat


class plateController(Sofa.Core.Controller):
    """
    This Controller simply takes the cableActuator's value
    and increases / decreases it depending on the pressed key ('+' or '-')
    """

    def __init__(self, mstate, delta=1.0, *a, **kw):
        """
        In the ctor, we want to first call the constructor for the parent class (trampoline)
        We then store the node we want to retrieve the actuator from in the class
        (Sofa.Core.Base.getContext() could also have been used here instead,
        or a link between aCableActuator and the controller could have been used too)
        """
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.mstate = mstate
        self.delta = delta
        return

    def onKeypressedEvent(self, e):
        """
        Events methods are named after the actual event names (Event::GetClassName() in C++),
        with a prepended "on" prefix. Thus, this Event is the KeypressedEvent class in C++
        The onXXXEvent method takes a dictionary as a parameter, containing the useful
        values stored in the event class, e.g. here, the pressed key
        """

        currentPos = self.mstate.position[0]

        # Move on Y axis
        if e["key"] == Sofa.constants.Key.KP_7:
            currentPos[1] += self.delta
        elif e["key"] == Sofa.constants.Key.KP_1:
            currentPos[1] -= self.delta
        elif e["key"] == Sofa.constants.Key.KP_9:  # Move on Z axis
            currentPos[2] += self.delta
        elif e["key"] == Sofa.constants.Key.KP_4:
            currentPos[2] -= self.delta

        self.mstate.position = [currentPos]

        return