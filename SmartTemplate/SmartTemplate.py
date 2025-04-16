import logging
import os

import vtk, qt, ctk, slicer
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from PythonQt import QtCore, QtGui
import vtk.util.numpy_support as numpy_support
import xml.etree.ElementTree as ET


# import SimpleITK as sitk
# import sitkUtils
import numpy as np
from time import sleep
import datetime

class SmartTemplate(ScriptedLoadableModule):

  def __init__(self, parent):
    ScriptedLoadableModule.__init__(self, parent)
    self.parent.title = "SmartTemplate"
    self.parent.categories = ["IGT"] 
    self.parent.dependencies = ["ZFrameRegistration", "ROS2"]  # TODO: add here list of module names that this module requires
    self.parent.contributors = ["Mariana Bernardes (BWH)"] 
    self.parent.helpText = """ This module is used to interact with the ROS2 packages for SmartTemplate needle guide robot. Uses ZFrameRegistration module for initialization of the ZTransform and SlicerROS2 module"""
    # TODO: replace with organization, grant and thanks
    self.parent.acknowledgementText = """ """

    # Additional initialization step after application startup is complete
    # TODO: include sample data and testing routines
    # slicer.app.connect("startupCompleted()", registerSampleData)

################################################################################################################################################
# Widget Class
################################################################################################################################################

class SmartTemplateWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
  """Uses ScriptedLoadableModuleWidget base class, available at:
  https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
  """
  # Called when the user opens the module the first time and the widget is initialized.
  def __init__(self, parent=None):
    ScriptedLoadableModuleWidget.__init__(self, parent)
    VTKObservationMixin.__init__(self)  # needed for parameter node observation
    self.logic = None
    self._parameterNode = None
    self._updatingGUIFromParameterNode = False

  ### Widget setup ###################################################################
  def setup(self):
    ScriptedLoadableModuleWidget.setup(self)
    print('Widget Setup')
    ####################################
    ##                                ##
    ## UI Components                  ##
    ##                                ##
    ####################################

    ## Robot collapsible button            
    ####################################
    robotCollapsibleButton = ctk.ctkCollapsibleButton()
    robotCollapsibleButton.text = 'Robot'
    self.layout.addWidget(robotCollapsibleButton)

    # Load robot button
    robotFormLayout = qt.QFormLayout(robotCollapsibleButton)
    self.loadButton = qt.QPushButton('Load SmartTemplate')
    self.loadButton.toolTip = 'Loads robot in 3DSlicer'
    self.loadButton.enabled = True
    robotFormLayout.addRow(self.loadButton)

    # ZTransform matrix
    registrationLayout = qt.QHBoxLayout()
    self.zTransformSelector = slicer.qMRMLNodeComboBox()
    self.zTransformSelector.nodeTypes = ['vtkMRMLLinearTransformNode']
    self.zTransformSelector.selectNodeUponCreation = True
    self.zTransformSelector.addEnabled = True
    self.zTransformSelector.removeEnabled = True
    self.zTransformSelector.noneEnabled = True
    self.zTransformSelector.showHidden = False
    self.zTransformSelector.showChildNodeTypes = False
    self.zTransformSelector.setMRMLScene(slicer.mrmlScene)
    self.zTransformSelector.setToolTip('Select the ZFrameToScanner Transform')
    ZTransformLabel = qt.QLabel('ZTransform:')
    registrationLayout.addWidget(ZTransformLabel)
    registrationLayout.addWidget(self.zTransformSelector)
    
    # Register robot button 
    self.registerButton = qt.QPushButton('Register SmartTemplate')
    self.registerButton.toolTip = 'Registers robot to scanner'
    self.registerButton.enabled = False
    registrationLayout.addWidget(self.registerButton)
    robotFormLayout.addRow(registrationLayout)

    jointsContainer = qt.QWidget()
    jointsContainer.setMinimumHeight(200)  # Set minimum height for the container
    # Robot joints
    self.jointsLayout = qt.QVBoxLayout(jointsContainer)
    robotFormLayout.addWidget(jointsContainer)

    # Robot position
    positionLayout = qt.QHBoxLayout()
    positionLabel = qt.QLabel('Robot coordinates:')
    self.positionRASTextbox = qt.QLineEdit('(---, ---, ---) RAS')
    self.positionRASTextbox.setReadOnly(True)
    self.positionRASTextbox.setStyleSheet('background-color: transparent; border: no border;')
    self.positionRASTextbox.toolTip = 'Robot current position (scanner frame)'
    self.positionRobotTextbox = qt.QLineEdit('(---, ---, ---) robot')
    self.positionRobotTextbox.setReadOnly(True)
    self.positionRobotTextbox.setStyleSheet('background-color: transparent; border: no border;')
    self.positionRobotTextbox.toolTip = 'Robot current position (robot frame)'
    positionLayout.addWidget(positionLabel)
    positionLayout.addWidget(self.positionRASTextbox)
    positionLayout.addWidget(self.positionRobotTextbox)
    timestampLabel = qt.QLabel('Timestamp:')
    self.timeStampTextbox = qt.QLineEdit('-- : -- : --.----')
    self.timeStampTextbox.setReadOnly(True)
    self.timeStampTextbox.setStyleSheet('background-color: transparent; border: no border;')
    self.timeStampTextbox.toolTip = 'Timestamp from last shape measurement'
    positionLayout.addWidget(timestampLabel)
    positionLayout.addWidget(self.timeStampTextbox)
    robotFormLayout.addRow(positionLayout)


    ## Planning collapsible button                 
    ####################################
    
    planningCollapsibleButton = ctk.ctkCollapsibleButton()
    planningCollapsibleButton.text = 'Planning'
    self.layout.addWidget(planningCollapsibleButton)   
    planningFormLayout = qt.QVBoxLayout(planningCollapsibleButton)
    
    # Points selection
    markupsLayout = qt.QFormLayout()
    planningFormLayout.addLayout(markupsLayout)
    self.pointListSelector = slicer.qSlicerSimpleMarkupsWidget()    
    self.pointListSelector.setMRMLScene(slicer.mrmlScene)
    self.pointListSelector.setNodeSelectorVisible(False)
    self.pointListSelector.markupsPlaceWidget().setPlaceMultipleMarkups(True)
    self.pointListSelector.defaultNodeColor = qt.QColor(170,0,0)
    self.pointListSelector.setMaximumHeight(90)
    self.pointListSelector.tableWidget().show()
    self.pointListSelector.toolTip = 'Select 2 points: TARGET and ENTRY'
    # self.pointListSelector.markupsPlaceWidget().setPlaceModePersistency(True)
    markupsLayout.addRow('Planned points:', self.pointListSelector)
    
    # SendPlan button 
    self.adjustEntryButton = qt.QPushButton('Align ENTRY to TARGET')
    self.adjustEntryButton.toolTip = 'Start targeting'
    self.adjustEntryButton.enabled = False
    markupsLayout.addRow('', self.adjustEntryButton)

## Insertion collapsible button                
    ####################################
    
    insertionCollapsibleButton = ctk.ctkCollapsibleButton()
    insertionCollapsibleButton.text = 'Insertion'    
    self.layout.addWidget(insertionCollapsibleButton)
    insertionFormLayout = qt.QFormLayout(insertionCollapsibleButton)

    commandButtonsLayout = qt.QHBoxLayout()
    self.homeButton = qt.QPushButton('HOME')
    self.homeButton.toolTip = 'Go to robot HOME position'
    self.homeButton.enabled = False
    commandButtonsLayout.addWidget(self.homeButton)

    self.retractButton = qt.QPushButton('RETRACT')
    self.retractButton.toolTip = 'Fully retract needle'
    self.retractButton.enabled = False
    commandButtonsLayout.addWidget(self.retractButton)
    
    self.alignButton = qt.QPushButton('ALIGN')
    self.alignButton.toolTip = 'Align robot to TARGET'
    self.alignButton.enabled = False
    commandButtonsLayout.addWidget(self.alignButton)

    self.approachButton = qt.QPushButton('APPROACH')
    self.approachButton.toolTip = 'Plate robot at ENTRY point'
    self.approachButton.enabled = True
    commandButtonsLayout.addWidget(self.approachButton)

    insertionFormLayout.addRow(commandButtonsLayout)

    self.layout.addStretch(1)
    
    ####################################
    ##                                ##
    ## UI Behavior                    ##
    ##                                ##
    ####################################

    # Internal variables
    self.isRobotLoaded = False  # Is the robot loaded?
    self.jointNames = None
    self.jointLimits = None
    self.jointValues = None
    
    # Initialize module logic
    self.logic = SmartTemplateLogic()

    # Make sure parameter node is initialized (needed for module reload)
    self.initializeParameterNode()
        
    # These connections ensure that we update parameter node when scene is closed
    self.addObserver(slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent, self.onSceneStartClose)
    self.addObserver(slicer.mrmlScene, slicer.mrmlScene.EndCloseEvent, self.onSceneEndClose)

    # These connections ensure that we synch jointValues with the logic
    self.addObserver(self.logic.jointValues, vtk.vtkCommand.ModifiedEvent, self.onJointsChange)
    self.addObserver(self.logic.robotPosition, vtk.vtkCommand.ModifiedEvent, self.onPositionChange)

    # These connections ensure that whenever user changes some settings on the GUI, that is saved in the MRML scene
    # (in the selected parameter node).
    self.zTransformSelector.connect('currentNodeChanged(vtkMRMLNode*)', self.updateParameterNodeFromGUI)
    
    # Connect Qt widgets to event calls
    self.loadButton.connect('clicked(bool)', self.loadRobot)
    self.registerButton.connect('clicked(bool)', self.registerRobot)
    self.adjustEntryButton.connect('clicked(bool)', self.adjustEntry)
    self.approachButton.connect('clicked(bool)', self.approachEntry)
    self.pointListSelector.connect('updateFinished()', self.onPointListChanged)

    # Initialize widget variables and updateGUI
    self.updateGUI()
    self.pointListSelector.setCurrentNode(self.logic.pointListNode)
    interactionNode = slicer.mrmlScene.GetNodeByID("vtkMRMLInteractionNodeSingleton")
    interactionNode.SetCurrentInteractionMode(interactionNode.ViewTransform)  # Instead of interactionNode.Place


  ### Widget functions ###################################################################
  # Called when the application closes and the module widget is destroyed.
  def cleanup(self):
    self.removeObservers()

  # Called each time the user opens this module.
  # Make sure parameter node exists and observed
  def enter(self):
    self.initializeParameterNode() 

  # Called each time the user opens a different module.
  # Do not react to parameter node changes (GUI will be updated when the user enters into the module)
  def exit(self):
    self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)

  # Called just before the scene is closed.
  # Parameter node will be reset, do not use it anymore
  def onSceneStartClose(self, caller, event):
    self.logic.closeConnection()    
    self.setParameterNode(None)

  # Called just after the scene is closed.
  # If this module is shown while the scene is closed then recreate a new parameter node immediately
  def onSceneEndClose(self, caller, event):
    if self.parent.isEntered:
      self.initializeParameterNode()
        
  # Ensure parameter node exists and observed
  # Parameter node stores all user choices in parameter values, node selections, etc.
  # so that when the scene is saved and reloaded, these settings are restored.
  def initializeParameterNode(self):
    # Load default parameters in logic module
    self.setParameterNode(self.logic.getParameterNode())
    # NOT DOING THIS: sometimes it changes the selection when going back between modules
    # # Select default input nodes if nothing is selected yet to save a few clicks for the user
    # if not self._parameterNode.GetNodeReference('ZTransform'):
    #   # Find first selectable transform
    #   zTransformNode = next((node for node in slicer.util.getNodesByClass('vtkMRMLLinearTransformNode') if node.GetSelectable()==1), None)
    #   if zTransformNode:
    #     self._parameterNode.SetNodeReferenceID('ZTransform', zTransformNode.GetID())
            
  # Set and observe parameter node.
  # Observation is needed because when the parameter node is changed then the GUI must be updated immediately.
  def setParameterNode(self, inputParameterNode):
    if inputParameterNode:
      self.logic.setDefaultParameters(inputParameterNode)
    # Unobserve previously selected parameter node and add an observer to the newly selected.
    # Changes of parameter node are observed so that whenever parameters are changed by a script or any other module
    # those are reflected immediately in the GUI.
    if self._parameterNode is not None and self.hasObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode):
        self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)
    self._parameterNode = inputParameterNode
    if self._parameterNode is not None:
        self.addObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)
    # Initial GUI update
    self.updateGUIFromParameterNode()

  # This method is called whenever parameter node is changed.
  # The module GUI is updated to show the current state of the parameter node.
  def updateGUIFromParameterNode(self, caller=None, event=None):
    if self._parameterNode is None or self._updatingGUIFromParameterNode:
      return
    # Make sure GUI changes do not call updateParameterNodeFromGUI (it could cause infinite loop)
    self._updatingGUIFromParameterNode = True
    # Update node selectors and input boxes and sliders
    self.zTransformSelector.setCurrentNode(self._parameterNode.GetNodeReference('ZTransform'))
    self.pointListSelector.setCurrentNode(self._parameterNode.GetNodeReference('Planning'))
    self.isRobotLoaded = self._parameterNode.GetParameter('RobotLoaded') == 'True'
    self.updateGUI()
    # All the GUI updates are done
    self._updatingGUIFromParameterNode = False

  # This method is called when the user makes any change in the GUI.
  # The changes are saved into the parameter node (so that they are restored when the scene is saved and loaded).
  def updateParameterNodeFromGUI(self, caller=None, event=None):
    if self._parameterNode is None or self._updatingGUIFromParameterNode:
      return
    # Modify all properties in a single batch
    wasModified = self._parameterNode.StartModify()  
    # Update paramenters_nodes
    self._parameterNode.SetNodeReferenceID('ZTransform', self.zTransformSelector.currentNodeID)
    # All paramenter_nodes updates are done
    self._parameterNode.EndModify(wasModified)
    
  # Called when the Point List is updated
  def onPointListChanged(self):    
    self.logic.addPlanningPoint(self.pointListSelector.currentNode())
    self.updateGUI()
  
  # Update the connection statusLabel  
  def onConnectionStatusChange(self, caller=None, event=None):
    self.updateGUI()

  # Update GUI buttons, markupWidget and connection statusLabel
  def updateGUI(self):
    # Check status
    zFrameSelected = (self.zTransformSelector.currentNode() is not None)
    pointsSelected = (self.logic.getNumberOfPoints(self.pointListSelector.currentNode()) == 2)
    robotRegistered = False if self.logic.mat_RobotToScanner is None else True

    # Update buttons accordingly
    self.loadButton.enabled = not self.isRobotLoaded
    self.registerButton.enabled = zFrameSelected and self.isRobotLoaded
    self.adjustEntryButton.enabled = pointsSelected and robotRegistered and self.isRobotLoaded
    
    # Add joints if not already added:
    if self.jointNames is None and self.isRobotLoaded:
      self.addJoints()
    # self.pointListSelector.placeActive(not pointsSelected)

  def format3DPoint(self, point, frame=None, decimals=2):
    formatted = f"({point[0]:.{decimals}f}, {point[1]:.{decimals}f}, {point[2]:.{decimals}f})"
    if frame:
        formatted += f" {frame}"
    return formatted

  def addJoints(self):
    (self.jointNames, self.jointLimits) = self.logic.getDefinedJoints()
    if self.jointNames is None:
      print('Could not load joints')
    else:
      # Initialize joint values with 0.0 
      self.jointValues = {joint_name: 0.0 for joint_name in self.jointNames}
      # Dictionaries to hold widgets
      self.sliders = {}
      self.text_boxes = {}
      self.current_value_boxes = {}  # For 'Current [mm]' textboxes
      for joint in self.jointNames:
          # Remove '_joint' suffix and capitalize the label
          joint_label = qt.QLabel(f'{joint.replace("_joint", "").capitalize()}')
          # Slider for current joint state
          slider = qt.QSlider(qt.Qt.Horizontal)
          limits = self.jointLimits[joint]
          slider.setMinimum(int(limits['min']))
          slider.setMaximum(int(limits['max']))
          slider.setEnabled(False)  # Non-editable
          slider.setValue(self.jointValues[joint])
          # Min and Max labels
          min_label = qt.QLabel(f"{limits['min']}")
          max_label = qt.QLabel(f"{limits['max']}")
          # Layout for slider and labels
          slider_layout = qt.QHBoxLayout()
          slider_layout.addWidget(min_label)
          slider_layout.addWidget(slider)
          slider_layout.addWidget(max_label)
          # Text box for current joint value (non-editable)
          current_value_box = qt.QLineEdit('0.0')
          current_value_box.setReadOnly(True)
          current_value_box.setStyleSheet("background: transparent; border: none; color: black;")
          current_value_box.setText(str(self.jointValues[joint]))
          current_value_label = qt.QLabel('Current [mm]:')
          # Layout for current joint values
          value_layout = qt.QHBoxLayout()
          value_layout.addWidget(current_value_label)
          value_layout.addWidget(current_value_box)
          # Layout for each joint
          joint_layout = qt.QVBoxLayout()
          joint_layout.addWidget(joint_label)
          joint_layout.addLayout(slider_layout)
          joint_layout.addLayout(value_layout)
          # Add the joint layout to the main joint controls layout
          self.jointsLayout.addLayout(joint_layout)
          # Add a horizontal separator line after each joint section
          separator_line = qt.QFrame()
          separator_line.setFrameShape(qt.QFrame.HLine)
          separator_line.setFrameShadow(qt.QFrame.Sunken)
          self.jointsLayout.addWidget(separator_line)
          # Save references
          self.sliders[joint] = slider
          self.current_value_boxes[joint] = current_value_box


  # Update sliders and text boxes with current joint values
  def onJointsChange(self, caller=None, event=None):
    for joint in self.jointNames:
      if caller.GetAttribute(joint) is None:
        return
      self.jointValues[joint] = float(caller.GetAttribute(joint))
      self.sliders[joint].setValue(self.jointValues[joint])
      self.current_value_boxes[joint].setText(f"{self.jointValues[joint]:.2f}")

  # Update text boxes with current position values
  def onPositionChange(self, caller=None, event=None):
    robotPositionXYZ = [0.0, 0.0, 0.0]
    self.logic.robotPosition.GetNthControlPointPosition(0, robotPositionXYZ)
    #print('Time: %s / Tip (RAS): %s' %(timestamp,tipCoordinates))
    if robotPositionXYZ is not None:
      self.positionRobotTextbox.setText(self.format3DPoint(robotPositionXYZ, 'XYZ'))
    else:
      self.positionRobotTextbox.setText('(---, ---, ---)')
    robotPositionRAS = [0.0, 0.0, 0.0]
    self.logic.robotPosition.GetNthControlPointPositionWorld(0, robotPositionRAS)
    if robotPositionRAS is not None:
      self.positionRASTextbox.setText(self.format3DPoint(robotPositionRAS, 'RAS'))
    else:
      self.positionRASTextbox.setText('(---, ---, ---)')
    self.timeStampTextbox.setText(self.logic.robotPositionTimestamp.GetText())

  def loadRobot(self):
    print('UI: loadRobot()')
    self.logic.loadRobot()
    self.updateGUI()

  def registerRobot(self):
    print('UI: registerRobot()')
    zTransformNode = self.zTransformSelector.currentNode()
    self.logic.registerRobot(zTransformNode)
    self.updateGUI()

  def adjustEntry(self):
    print('UI: adjustEntry()')
    # Get current transform node
    pointListNode = self.pointListSelector.currentNode()
    self.logic.alignEntryToTarget(pointListNode)
    self.updateGUI()

  def approachEntry(self):
    print('UI: approachEntry()')
    # Get current transform node
    pointListNode = self.pointListSelector.currentNode()
    self.logic.approachEntry(pointListNode)
    self.updateGUI()

   
################################################################################################################################################
# Logic Class
################################################################################################################################################

class SmartTemplateLogic(ScriptedLoadableModuleLogic):

  def __init__(self):
    ScriptedLoadableModuleLogic.__init__(self)
    self.cliParamNode = None
    print('Logic: __init__')

    # SlicerROS2 module internal variables
    self.rosLogic = slicer.util.getModuleLogic('ROS2')
    self.rosNode = self.rosLogic.GetDefaultROS2Node()
    self.robotNode = self.rosNode.GetRobotNodeByName('smart_template')
    self.pubWorld = self.rosNode.GetPublisherNodeByTopic('/world_pose')
    self.pubGoal = self.rosNode.GetPublisherNodeByTopic('/desired_position')
    self.pubEntry = self.rosNode.GetPublisherNodeByTopic('/planning/skin_entry')
    self.pubTarget = self.rosNode.GetPublisherNodeByTopic('/planning/target')
    self.subJoints = self.rosNode.GetSubscriberNodeByTopic('/joint_states')
    self.subPosition = self.rosNode.GetSubscriberNodeByTopic('/stage/state/guide_pose')
    if self.subJoints is not None:
      self.observeJoints = self.subJoints.AddObserver('ModifiedEvent', self.onSubJointsMsg)
    if self.subPosition is not None:
      self.observePosition = self.subPosition.AddObserver('ModifiedEvent', self.onSubPositionMsg)  

    self.link_names = None
    self.joint_names = None
    self.joint_limits = None
    self.desired_position = None

    self.mat_RobotToZFrame = None
    self.mat_ZFrameToScanner = None
    self.mat_RobotToScanner =  None
    self.mat_ScannerToRobot =  None

    # If robot node is available, make sure the /robot_state_publisher is up
    if self.robotNode:
      paramNode = self.rosNode.GetParameterNodeByNode('/robot_state_publisher')
      if self.extractRobotParameters(paramNode) is False:
        print('Robot missing /robot_state_publisher')

    # MRML Objects for robot position
    # Robot to Scanner Transform node
    self.robotToScannerTransformNode = slicer.util.getFirstNodeByClassByName('vtkMRMLLinearTransformNode','RobotToScannerTransform')
    if self.robotToScannerTransformNode is None:
        self.robotToScannerTransformNode = slicer.vtkMRMLLinearTransformNode()
        self.robotToScannerTransformNode.SetName('RobotToScannerTransform')
        slicer.mrmlScene.AddNode(self.robotToScannerTransformNode)
    # Robot position message header
    self.robotPositionTimestamp = slicer.util.getFirstNodeByName('RobotPositionTimestamp', className='vtkMRMLTextNode')
    if self.robotPositionTimestamp is None:
      self.robotPositionTimestamp = slicer.vtkMRMLTextNode()
      self.robotPositionTimestamp.SetName('RobotPositionTimestamp')
      slicer.mrmlScene.AddNode(self.robotPositionTimestamp)
    # Robot position markups node
    self.robotPosition = slicer.util.getFirstNodeByName('RobotPosition', className='vtkMRMLMarkupsFiducialNode')
    if self.robotPosition is None:
      self.robotPosition = slicer.vtkMRMLMarkupsFiducialNode()
      self.robotPosition.SetName('RobotPosition')
      slicer.mrmlScene.AddNode(self.robotPosition)
    self.initializeRobotPositionMarkup()

    # Create PointList node for planning
    self.pointListNode = slicer.util.getFirstNodeByName('Planning', className='vtkMRMLMarkupsFiducialNode')
    if self.pointListNode is None:
        self.pointListNode = slicer.vtkMRMLMarkupsFiducialNode()
        self.pointListNode.SetName('Planning')
        slicer.mrmlScene.AddNode(self.pointListNode)
    displayNodePointList = self.pointListNode.GetDisplayNode()
    if displayNodePointList:
      displayNodePointList.SetGlyphScale(1.5)   
    
    # Create ScriptedModule node for joint values
    self.jointValues = slicer.util.getFirstNodeByName('JointValues', className='vtkMRMLScriptedModuleNode')
    if self.jointValues is None:
      self.jointValues = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLScriptedModuleNode', 'JointValues')

  ### Logic functions ###################################################################

  def initializeRobotPositionMarkup(self):
    # Ensure there is only one control point
    if self.robotPosition.GetNumberOfControlPoints() > 1:
        self.robotPosition.RemoveAllControlPoints()
    # If no control point exists, add one at (0,0,0)
    if self.robotPosition.GetNumberOfControlPoints() == 0:
        self.robotPosition.AddControlPoint(vtk.vtkVector3d(0, 0, 0), "")
    # Ensure tip is labeled, locked and one-point only
    self.robotPosition.SetNthControlPointLabel(0, "")
    self.robotPosition.SetLocked(True)
    self.robotPosition.SetFixedNumberOfControlPoints(True)           
    displayNode = self.robotPosition.GetDisplayNode()
    if displayNode:
        displayNode.SetGlyphScale(1.5)  # 1% glyph size and color
        displayNode.SetSelectedColor(0.667, 1.0, 0.498)
    else:
        # If the display node does not exist, create it and set the glyph size and color
        self.robotPosition.CreateDefaultDisplayNodes()
        self.robotPosition.GetDisplayNode().SetGlyphScale(1.5)
        displayNode.SetSelectedColor(0.667, 1.0, 0.498)
          # TODO: Set mat_RobotToScanner Transform 
    if self.robotToScannerTransformNode:
      self.robotPosition.SetAndObserveTransformNodeID(self.robotToScannerTransformNode.GetID())

  # Initialize module parameter node with default settings
  def setDefaultParameters(self, parameterNode):
    # self.initialize()
    if not parameterNode.GetParameter('RobotLoaded'):
      if self.joint_names is None:
        isRobotLoaded = 'False'
      else:
        isRobotLoaded = 'True'
      parameterNode.SetParameter('RobotLoaded', isRobotLoaded)  
    if not parameterNode.GetNodeReference('Planning'):
      parameterNode.SetNodeReferenceID('Planning', self.pointListNode.GetID())

  # Print vtkMatrix4x4 (for debugging)
  def printVtkMatrix4x4(self, matrix4x4, name=''):
    print(name)
    for i in range(4):
      for j in range(4):
        print(matrix4x4.GetElement(i, j), end=" ")
      print()

  def formatTimestampToLocalTimeText(self, sec, nanosec):
    # Convert to seconds as float
    timestamp_sec = sec + nanosec / 1e9
    # Convert to local datetime
    dt = datetime.datetime.fromtimestamp(timestamp_sec)
    # Format: HH:MM:SS.mmm
    return dt.strftime('%H:%M:%S.') + f"{int(dt.microsecond / 1000):03d}"
  
  # Callback for /joints_state messages
  def onSubJointsMsg(self, caller=None, event=None):
    joints_msg = self.subJoints.GetLastMessage()
    # Extract joint names and positions from the message
    msg_joint_names = joints_msg.GetName()
    msg_joint_values = joints_msg.GetPosition()
    for joint_name, joint_value in zip(msg_joint_names, msg_joint_values):
      self.jointValues.SetAttribute(joint_name, str(1000*joint_value))  # Triggers observer
    self.jointValues.Modified()

  # Callback for /joints_state messages
  def onSubPositionMsg(self, caller=None, event=None):
    position_msg = self.subPosition.GetLastMessage()
    # Update timestamp string node
    stamp = position_msg.GetHeader().GetStamp()
    timestamp = self.formatTimestampToLocalTimeText(stamp.GetSec(), stamp.GetNanosec())
    self.robotPositionTimestamp.SetText(timestamp)
    self.robotPositionTimestamp.Modified()
    # Update position markups node
    point =  position_msg.GetPoint()
    position = [point.GetX(), point.GetY(), point.GetZ()]
    self.robotPosition.SetNthControlPointPosition(0, position)
    self.robotPosition.Modified()

  # Wait for robot models to be ready
  def onRobotModelReady(self, caller=None, event=None):
    N_model = self.robotNode.GetNumberOfNodeReferences('model')
    if(N_model >= len(self.link_names)):
      # print('Robot model %i available' %N_model)
      model_name = self.robotNode.GetNthNodeReferenceID('model', N_model-1)
      needle_node = slicer.mrmlScene.GetNodeByID(model_name)
      display_node = needle_node.GetDisplayNode()
      if display_node is None:
        needle_node.CreateDefaultDisplayNodes()
        display_node = needle_node.GetDisplayNode()
      # Change display settings
      display_node.SetVisibility2D(True)
      display_node.SetColor(1.0, 0.0, 1.0)
      display_node.SetSliceIntersectionThickness(2)
      caller.RemoveObserver(self.observeRobotModel)

  # Wait for robot_description parameter and then define link_names, joint_names and joint_limits  
  @vtk.calldata_type(vtk.VTK_OBJECT)
  def onRobotDescriptioReady(self, caller, calldata):
    if caller.GetMonitoredNodeName() == '/robot_state_publisher':
      print('Robot description available')
      if self.extractRobotParameters(caller):
        moduleParameterNode = self.getParameterNode()     # Update module parameter to help synch logic and gui
        moduleParameterNode.SetParameter('RobotLoaded', 'True')
        caller.RemoveObserver(self.observeRobotDescription)
        self.observeRobotModel = self.robotNode.AddObserver(slicer.vtkMRMLROS2RobotNode.ReferenceAddedEvent, self.onRobotModelReady)

  # Get last lookupNode if available    
  def getLastLookupNode(self):
    N_lookup = self.robotNode.GetNumberOfNodeReferences('lookup')
    if(N_lookup >= len(self.link_names)):
      lookup_name = self.robotNode.GetNthNodeReferenceID('lookup', N_lookup-1)
      return slicer.mrmlScene.GetNodeByID(lookup_name)
    else:
      return None

  # Get info from robot_description
  def extractRobotParameters(self, paramNode):
    if paramNode is None:
      print('Parameter node is not available')
      return False
    if paramNode.IsParameterSet('robot_description'):
      robot_description = paramNode.GetParameterAsString('robot_description')
      print('Reading robot description...')
      # Parse URDF
      root = ET.fromstring(robot_description)
      link_names = []
      joint_names = []
      joint_limits = {}
      for link in root.findall('link'):
          link_name = link.get('name')
          link_names.append(link_name)
      for joint in root.findall('joint'):
          joint_name = joint.get('name')
          limit = joint.find('limit')
          if limit is not None:
              lower = 1000*float(limit.get('lower', '0.0'))
              upper = 1000*float(limit.get('upper', '0.0'))
              joint_names.append(joint_name)
              joint_limits[joint_name] = {'min': lower, 'max': upper}
      # Extract zframe_pose
      custom_parameters = root.find('custom_parameters')
      if custom_parameters is not None:
        zframe_pose_element = custom_parameters.find('zframe_pose')
        if zframe_pose_element is not None:
          zframe_pose = zframe_pose_element.get('value').strip()
          # print('ZFrameToRobot (Homogeneous Matrix):', zframe_pose)
          # Convert to a 4x4 matrix
          rows = zframe_pose.split('       ')  # Split by rows
          matrix_values = [list(map(float, row.split())) for row in rows]
          # Scale the translation elements by 1000
          matrix_values[0][3] *= 1000  # x translation
          matrix_values[1][3] *= 1000  # y translation
          matrix_values[2][3] *= 1000  # z translation          
          # Create vtkMatrix4x4 and populate it
          mat_ZFrameToRobot = vtk.vtkMatrix4x4()
          self.mat_RobotToZFrame = vtk.vtkMatrix4x4()
          for i in range(4):
            for j in range(4):
              mat_ZFrameToRobot.SetElement(i, j, matrix_values[i][j])
          self.mat_RobotToZFrame.DeepCopy(mat_ZFrameToRobot )
          self.mat_RobotToZFrame.Invert()
          self.printVtkMatrix4x4(mat_ZFrameToRobot, 'ZFrameToRobot = ')
        else:
          print('zframe_pose not found in custom_parameters.')
      else:
        print('custom_parameters section not found.')
      # Store extracted data in the class attributes
      self.link_names = link_names
      self.joint_names = joint_names
      self.joint_limits = joint_limits
      print('Robot links = %s' %self.link_names)
      print('Robot joints = %s' %self.joint_names)
      return True
    else:
      print('Robot description not available')
      return False

  def getJointValues(self):
    if self.joints is None:
      return None
    else:
      return self.joints

  def getDefinedJoints(self):
    if self.joint_names is None:
      return (None, None)
    else:
      return (self.joint_names, self.joint_limits)

  def loadRobot(self):
    # Create robot node and publisher to /world_pose message
    if self.robotNode is None:
      self.robotNode = self.rosNode.CreateAndAddRobotNode('smart_template','/robot_state_publisher','robot_description', 'world', '' ) 
      paramNode = slicer.util.getNode(self.robotNode.GetNodeReferenceID('parameter'))
      self.observeRobotDescription = paramNode.AddObserver(slicer.vtkMRMLROS2ParameterNode.ParameterModifiedEvent, self.onRobotDescriptioReady)
      if self.pubWorld is None:
          self.pubWorld = self.rosNode.CreateAndAddPublisherNode('TransformStamped', '/world_pose')
      if self.pubGoal is None:
        self.pubGoal = self.rosNode.CreateAndAddPublisherNode('Point', '/desired_position')
      if self.pubEntry is None:
        self.pubEntry = self.rosNode.CreateAndAddPublisherNode('Point', '/planning/skin_entry')
      if self.pubTarget is None:
        self.pubTarget = self.rosNode.CreateAndAddPublisherNode('Point', '/planning/target')
      if self.subJoints is None:
        self.subJoints = self.rosNode.CreateAndAddSubscriberNode('JointState', '/joint_states')
        self.observeJoints = self.subJoints.AddObserver('ModifiedEvent', self.onSubJointsMsg)
      if self.subPosition is None:
        self.subPosition = self.rosNode.CreateAndAddSubscriberNode('PointStamped', '/stage/state/guide_pose')
        self.observePosition = self.subPosition.AddObserver('ModifiedEvent', self.onSubPositionMsg)  
      print('Robot initialized')
    else:
      # Should not enter here
      print('Robot already intialized')
    return True

  def registerRobot(self, ZFrameToScanner):
    if self.robotNode is None:
      print('Initialize robot first')
      return False
    if self.mat_RobotToScanner is None:
      self.mat_RobotToScanner =  vtk.vtkMatrix4x4()
      self.mat_ZFrameToScanner = vtk.vtkMatrix4x4()
      self.mat_ScannerToRobot =  vtk.vtkMatrix4x4()
    ZFrameToScanner.GetMatrixTransformToWorld(self.mat_ZFrameToScanner)
    vtk.vtkMatrix4x4.Multiply4x4(self.mat_ZFrameToScanner, self.mat_RobotToZFrame, self.mat_RobotToScanner)
    vtk.vtkMatrix4x4.Invert(self.mat_RobotToScanner, self.mat_ScannerToRobot)
    # Update robotToScanner transform
    self.robotToScannerTransformNode.SetMatrixTransformToParent(self.mat_RobotToScanner)   
    # Publish robotToScanner to world_pose broadcaster
    # TODO: Replace by a static broadcaster once it becomes available in SlicerROS2
    world_msg = self.pubWorld.GetBlankMessage()
    world_msg.SetTransform(self.mat_RobotToScanner)
    self.pubWorld.Publish(world_msg)
    self.printVtkMatrix4x4(self.mat_RobotToScanner, '\world_pose')
    return True

  def getNumberOfPoints(self, pointListNode):
    if pointListNode is not None:
      return pointListNode.GetNumberOfDefinedControlPoints()
    else: 
      return 0
  
  def addPlanningPoint(self, pointListNode):
    if pointListNode is not None:
      N = pointListNode.GetNumberOfControlPoints()
      if N>=1:
        pointListNode.SetNthControlPointLabel(0, 'TARGET')
      if N>=2:
        pointListNode.SetNthControlPointLabel(1, 'ENTRY')
      if pointListNode.GetNumberOfDefinedControlPoints()>=2:
        pointListNode.SetFixedNumberOfControlPoints(2)
    
  # Place robot at the skin entry point
  def approachEntry(self, pointListNode):
    # Get entry point in scanner coordinates
    entry_scanner = [*pointListNode.GetNthControlPointPosition(pointListNode.GetControlPointIndexByLabel('ENTRY')), 1.0]
    # Calculate entry point in robot coordinates
    entry = self.mat_ScannerToRobot.MultiplyPoint(entry_scanner)
    self.sendDesiredPosition(entry)
    return True

  def isEntryAligned(self, pointListNode):
    # Get Points in scanner coordinates
    entry_scanner = [*pointListNode.GetNthControlPointPosition(pointListNode.GetControlPointIndexByLabel('ENTRY')), 1.0]
    target_scanner = [*pointListNode.GetNthControlPointPosition(pointListNode.GetControlPointIndexByLabel('TARGET')), 1.0]
    # Calculate Points in robot coordinates
    target = self.mat_ScannerToRobot.MultiplyPoint(target_scanner)
    entry = self.mat_ScannerToRobot.MultiplyPoint(entry_scanner)
    # Align entry with target (in robot coordinates)
    aligned_entry = [target[0], entry[1], target[2], 1.0]
    if entry == aligned_entry:
      return True
    else:
      return False
    
  def alignEntryToTarget(self, pointListNode):
    # Get Points in scanner coordinates
    entry_scanner = [*pointListNode.GetNthControlPointPosition(pointListNode.GetControlPointIndexByLabel('ENTRY')), 1.0]
    target_scanner = [*pointListNode.GetNthControlPointPosition(pointListNode.GetControlPointIndexByLabel('TARGET')), 1.0]
    # Calculate Points in robot coordinates
    target = self.mat_ScannerToRobot.MultiplyPoint(target_scanner)
    entry = self.mat_ScannerToRobot.MultiplyPoint(entry_scanner)
    # Align entry with target (in robot coordinates)
    entry = [target[0], entry[1], target[2], 1.0]
    # Update entry scanner in markups list (to align with target)
    entry_scanner = self.mat_RobotToScanner.MultiplyPoint(entry)
    entry_scanner = entry_scanner[0:3]
    pointListNode.SetNthControlPointPosition(pointListNode.GetControlPointIndexByLabel('ENTRY'), entry_scanner)
    return True

  def sendPlanning(self, entry, target):
    # Publish entry message
    entry_msg = self.pubEntry.GetBlankMessage()
    entry_msg.SetX(entry[0])
    entry_msg.SetY(entry[1])
    entry_msg.SetZ(entry[2])
    self.pubEntry.Publish(entry_msg)
    print('Sent entry point: %s' %(entry))
    # Publish target message
    target_msg = self.pubTarget.GetBlankMessage()
    target_msg.SetX(target[0])
    target_msg.SetY(target[1])
    target_msg.SetZ(target[2])
    self.pubTarget.Publish(target_msg)
    print('Sent target: %s' %(target))

  def sendDesiredPosition(self, goal):
    # Publish entry message
    self.desired_position = goal
    goal_msg = self.pubGoal.GetBlankMessage()
    goal_msg.SetX(self.desired_position[0])
    goal_msg.SetY(self.desired_position[1])
    goal_msg.SetZ(self.desired_position[2])
    self.pubGoal.Publish(goal_msg)
    print(self.desired_position)