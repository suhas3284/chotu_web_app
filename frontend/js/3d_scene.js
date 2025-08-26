/**
 * 3D Robot Scene Manager
 * Creates and manages a simplified 6-DOF robot arm visualization
 */

class Robot3DScene {
    constructor(containerId, THREE) {
        console.log('Robot3DScene constructor called with THREE:', THREE);
        this.THREE = THREE; // Store THREE reference
        this.container = document.getElementById(containerId);
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.robot = null;
        this.joints = [];
        this.links = [];
        this.controls = null;
        
        if (!this.THREE) {
            throw new Error('THREE object is required for Robot3DScene initialization');
        }
        
        this.init();
        this.createRobotModel();
        this.setupLights();
        this.setupEnvironment();
        this.animate();
    }
    
    init() {
        console.log('Initializing 3D scene...');
        console.log('Container:', this.container);
        console.log('Container dimensions:', this.container.clientWidth, 'x', this.container.clientHeight);
        console.log('THREE object available:', !!this.THREE);
        
        // Create scene
        this.scene = new this.THREE.Scene();
        this.scene.background = new this.THREE.Color(0x1a1a1a);
        console.log('Scene created');
        
        // Create camera with better positioning for robot view
        this.camera = new this.THREE.PerspectiveCamera(
            60, // Reduced FOV for better perspective
            this.container.clientWidth / this.container.clientHeight, 
            0.1, 
            1000
        );
        // Position camera to better view the robot
        this.camera.position.set(0.8, 0.6, 0.8);
        this.camera.lookAt(0, 0.3, 0); // Look at center of robot
        console.log('Camera created at position:', this.camera.position);
        
        // Create renderer with better quality
        this.renderer = new this.THREE.WebGLRenderer({ 
            antialias: true,
            alpha: true,
            preserveDrawingBuffer: true
        });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = this.THREE.PCFSoftShadowMap;
        this.renderer.outputColorSpace = this.THREE.SRGBColorSpace;
        console.log('Renderer created with size:', this.renderer.domElement.width, 'x', this.renderer.domElement.height);
        
        // Add renderer to container
        this.container.appendChild(this.renderer.domElement);
        console.log('Renderer added to container');
        
        // Handle window resize
        window.addEventListener('resize', () => this.onWindowResize(), false);
        
        console.log('3D scene initialization complete');
    }
    
    createRobotModel() {
        console.log('Creating realistic robot model...');
        
        // Create robot group
        this.robot = new this.THREE.Group();
        this.scene.add(this.robot);
        console.log('Robot group created and added to scene');
        
        // Robot dimensions based on actual URDF specifications
        const dimensions = {
            // Base dimensions
            baseHeight: 0.04,
            baseRadius: 0.1,
            
            // Joint dimensions (from URDF analysis)
            joint1Height: 0.15756,  // Height from base to joint1
            joint2Offset: -0.001,   // Offset from joint1 to joint2
            joint3Offset: -0.1104,  // X offset from joint2 to joint3
            joint4Offset: -0.096,   // X offset from joint3 to joint4
            joint5Offset: -0.07318, // Y offset from joint4 to joint5
            joint6Offset: 0.0456,   // Y offset from joint5 to joint6
            joint7Offset: -0.012,   // Z offset from joint6 to joint7
            
            // Visual dimensions
            jointRadius: 0.03,
            linkWidth: 0.02,
            jointHeight: 0.04
        };
        
        console.log('Realistic robot dimensions:', dimensions);
        
        // Create base (wider, more realistic)
        const baseGeometry = new this.THREE.CylinderGeometry(dimensions.baseRadius, dimensions.baseRadius, dimensions.baseHeight, 16);
        const baseMaterial = new this.THREE.MeshPhongMaterial({ 
            color: 0x2c3e50,
            metalness: 0.3,
            roughness: 0.7
        });
        const base = new this.THREE.Mesh(baseGeometry, baseMaterial);
        base.position.y = dimensions.baseHeight / 2;
        base.castShadow = true;
        base.receiveShadow = true;
        this.robot.add(base);
        
        // Create joint1 (base rotation) - Blue
        const joint1Geometry = new this.THREE.CylinderGeometry(dimensions.jointRadius, dimensions.jointRadius, dimensions.jointHeight, 16);
        const joint1Material = new this.THREE.MeshPhongMaterial({ 
            color: 0x3498db,
            metalness: 0.5,
            roughness: 0.3
        });
        const joint1 = new this.THREE.Mesh(joint1Geometry, joint1Material);
        joint1.position.y = dimensions.baseHeight + dimensions.joint1Height / 2;
        joint1.castShadow = true;
        this.robot.add(joint1);
        this.joints.push(joint1);
        
        // Create link1 (connecting joint1 to joint2)
        const link1Geometry = new this.THREE.CylinderGeometry(dimensions.linkWidth, dimensions.linkWidth, dimensions.joint1Height, 8);
        const link1Material = new this.THREE.MeshPhongMaterial({ 
            color: 0xe74c3c,
            metalness: 0.4,
            roughness: 0.6
        });
        const link1 = new this.THREE.Mesh(link1Geometry, link1Material);
        link1.position.y = dimensions.baseHeight + dimensions.joint1Height / 2;
        link1.castShadow = true;
        this.robot.add(link1);
        this.links.push(link1);
        
        // Create joint2 - Red
        const joint2Geometry = new this.THREE.CylinderGeometry(dimensions.jointRadius, dimensions.jointRadius, dimensions.jointHeight, 16);
        const joint2Material = new this.THREE.MeshPhongMaterial({ 
            color: 0xe74c3c,
            metalness: 0.5,
            roughness: 0.3
        });
        const joint2 = new this.THREE.Mesh(joint2Geometry, joint2Material);
        joint2.position.y = dimensions.baseHeight + dimensions.joint1Height + dimensions.joint2Offset / 2;
        joint2.castShadow = true;
        this.robot.add(joint2);
        this.joints.push(joint2);
        
        // Create link2 (connecting joint2 to joint3)
        const link2Geometry = new this.THREE.CylinderGeometry(dimensions.linkWidth, dimensions.linkWidth, Math.abs(dimensions.joint3Offset), 8);
        const link2Material = new this.THREE.MeshPhongMaterial({ 
            color: 0xf39c12,
            metalness: 0.4,
            roughness: 0.6
        });
        const link2 = new this.THREE.Mesh(link2Geometry, link2Material);
        link2.position.x = dimensions.joint3Offset / 2;
        link2.position.y = dimensions.baseHeight + dimensions.joint1Height + dimensions.joint2Offset;
        link2.rotation.z = Math.PI / 2; // Rotate to horizontal
        link2.castShadow = true;
        this.robot.add(link2);
        this.links.push(link2);
        
        // Create joint3 - Orange
        const joint3Geometry = new this.THREE.CylinderGeometry(dimensions.jointRadius, dimensions.jointRadius, dimensions.jointHeight, 16);
        const joint3Material = new this.THREE.MeshPhongMaterial({ 
            color: 0xf39c12,
            metalness: 0.5,
            roughness: 0.3
        });
        const joint3 = new this.THREE.Mesh(joint3Geometry, joint3Material);
        joint3.position.x = dimensions.joint3Offset;
        joint3.position.y = dimensions.baseHeight + dimensions.joint1Height + dimensions.joint2Offset;
        joint3.castShadow = true;
        this.robot.add(joint3);
        this.joints.push(joint3);
        
        // Create link3 (connecting joint3 to joint4)
        const link3Geometry = new this.THREE.CylinderGeometry(dimensions.linkWidth, dimensions.linkWidth, Math.abs(dimensions.joint4Offset), 8);
        const link3Material = new this.THREE.MeshPhongMaterial({ 
            color: 0x9b59b6,
            metalness: 0.4,
            roughness: 0.6
        });
        const link3 = new this.THREE.Mesh(link3Geometry, link3Material);
        link3.position.x = dimensions.joint3Offset + dimensions.joint4Offset / 2;
        link3.position.y = dimensions.baseHeight + dimensions.joint1Height + dimensions.joint2Offset;
        link3.rotation.z = Math.PI / 2; // Rotate to horizontal
        link3.castShadow = true;
        this.robot.add(link3);
        this.links.push(link3);
        
        // Create joint4 - Purple
        const joint4Geometry = new this.THREE.CylinderGeometry(dimensions.jointRadius, dimensions.jointRadius, dimensions.jointHeight, 16);
        const joint4Material = new this.THREE.MeshPhongMaterial({ 
            color: 0x9b59b6,
            metalness: 0.5,
            roughness: 0.3
        });
        const joint4 = new this.THREE.Mesh(joint4Geometry, joint4Material);
        joint4.position.x = dimensions.joint3Offset + dimensions.joint4Offset;
        joint4.position.y = dimensions.baseHeight + dimensions.joint1Height + dimensions.joint2Offset;
        joint4.castShadow = true;
        this.robot.add(joint4);
        this.joints.push(joint4);
        
        // Create link4 (connecting joint4 to joint5)
        const link4Geometry = new this.THREE.CylinderGeometry(dimensions.linkWidth, dimensions.linkWidth, Math.abs(dimensions.joint5Offset), 8);
        const link4Material = new this.THREE.MeshPhongMaterial({ 
            color: 0x1abc9c,
            metalness: 0.4,
            roughness: 0.6
        });
        const link4 = new this.THREE.Mesh(link4Geometry, link4Material);
        link4.position.x = dimensions.joint3Offset + dimensions.joint4Offset;
        link4.position.y = dimensions.baseHeight + dimensions.joint1Height + dimensions.joint2Offset + dimensions.joint5Offset / 2;
        link4.rotation.x = Math.PI / 2; // Rotate to vertical
        link4.castShadow = true;
        this.robot.add(link4);
        this.links.push(link4);
        
        // Create joint5 - Teal
        const joint5Geometry = new this.THREE.CylinderGeometry(dimensions.jointRadius, dimensions.jointRadius, dimensions.jointHeight, 16);
        const joint5Material = new this.THREE.MeshPhongMaterial({ 
            color: 0x9b59b6,
            metalness: 0.5,
            roughness: 0.3
        });
        const joint5 = new this.THREE.Mesh(joint5Geometry, joint5Material);
        joint5.position.x = dimensions.joint3Offset + dimensions.joint4Offset;
        joint5.position.y = dimensions.baseHeight + dimensions.joint1Height + dimensions.joint2Offset + dimensions.joint5Offset;
        joint5.castShadow = true;
        this.robot.add(joint5);
        this.joints.push(joint5);
        
        // Create link5 (connecting joint5 to joint6)
        const link5Geometry = new this.THREE.CylinderGeometry(dimensions.linkWidth, dimensions.linkWidth, Math.abs(dimensions.joint6Offset), 8);
        const link5Material = new this.THREE.MeshPhongMaterial({ 
            color: 0xe67e22,
            metalness: 0.4,
            roughness: 0.6
        });
        const link5 = new this.THREE.Mesh(link5Geometry, link5Material);
        link5.position.x = dimensions.joint3Offset + dimensions.joint4Offset;
        link5.position.y = dimensions.baseHeight + dimensions.joint1Height + dimensions.joint2Offset + dimensions.joint5Offset + dimensions.joint6Offset / 2;
        link5.rotation.x = Math.PI / 2; // Rotate to vertical
        link5.castShadow = true;
        this.robot.add(link5);
        this.links.push(link5);
        
        // Create joint6 (end effector) - Orange
        const joint6Geometry = new this.THREE.CylinderGeometry(dimensions.jointRadius, dimensions.jointRadius, dimensions.jointHeight, 16);
        const joint6Material = new this.THREE.MeshPhongMaterial({ 
            color: 0xe67e22,
            metalness: 0.5,
            roughness: 0.3
        });
        const joint6 = new this.THREE.Mesh(joint6Geometry, joint6Material);
        joint6.position.x = dimensions.joint3Offset + dimensions.joint4Offset;
        joint6.position.y = dimensions.baseHeight + dimensions.joint1Height + dimensions.joint2Offset + dimensions.joint5Offset + dimensions.joint6Offset;
        joint6.castShadow = true;
        this.robot.add(joint6);
        this.joints.push(joint6);
        
        // Add end effector marker (gripper representation)
        const endEffectorGeometry = new this.THREE.ConeGeometry(dimensions.jointRadius * 0.8, dimensions.jointRadius * 1.5, 8);
        const endEffectorMaterial = new this.THREE.MeshPhongMaterial({ 
            color: 0xf1c40f,
            metalness: 0.6,
            roughness: 0.2
        });
        const endEffector = new this.THREE.Mesh(endEffectorGeometry, endEffectorMaterial);
        endEffector.position.x = dimensions.joint3Offset + dimensions.joint4Offset;
        endEffector.position.y = dimensions.baseHeight + dimensions.joint1Height + dimensions.joint2Offset + dimensions.joint5Offset + dimensions.joint6Offset + dimensions.joint7Offset;
        endEffector.castShadow = true;
        this.robot.add(endEffector);
        
        console.log('Realistic robot model creation complete');
        console.log('Total joints created:', this.joints.length);
        console.log('Total links created:', this.links.length);
        console.log('Robot object:', this.robot);
    }
    setupLights() {
        // Ambient light
        const ambientLight = new this.THREE.AmbientLight(0x404040, 0.6);
        this.scene.add(ambientLight);
        
        // Directional light
        const directionalLight = new this.THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(5, 5, 5);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        this.scene.add(directionalLight);
        
        // Point light for better illumination
        const pointLight = new this.THREE.PointLight(0xffffff, 0.5);
        pointLight.position.set(-5, 5, -5);
        this.scene.add(pointLight);
    }
    
    setupEnvironment() {
        console.log('Setting up environment...');
        
        // Add grid
        const gridHelper = new this.THREE.GridHelper(4, 20, 0x444444, 0x222222);
        this.scene.add(gridHelper);
        console.log('Grid added to scene');
        
        // Add coordinate axes
        const axesHelper = new this.THREE.AxesHelper(1);
        this.scene.add(axesHelper);
        console.log('Coordinate axes added to scene');
        
        // Add some environment objects
        const groundGeometry = new this.THREE.PlaneGeometry(10, 10);
        const groundMaterial = new this.THREE.MeshPhongMaterial({ color: 0x333333 });
        const ground = new this.THREE.Mesh(groundGeometry, groundMaterial);
        ground.rotation.x = -Math.PI / 2;
        ground.receiveShadow = true;
        this.scene.add(ground);
        console.log('Ground plane added to scene');
        
        console.log('Environment setup complete');
    }
    
    updateRobotState(jointPositions) {
        if (!jointPositions || jointPositions.length !== 6) {
            console.warn('Invalid joint positions:', jointPositions);
            return;
        }
        
        // Update joint rotations based on real robot data
        // Joint 1: Base rotation (Y-axis)
        if (this.joints[0]) {
            this.joints[0].rotation.y = jointPositions[0];
        }
        
        // Joint 2: Shoulder rotation (Y-axis)
        if (this.joints[1]) {
            this.joints[1].rotation.y = jointPositions[1];
        }
        
        // Joint 3: Elbow rotation (Y-axis)
        if (this.joints[2]) {
            this.joints[2].rotation.y = jointPositions[2];
        }
        
        // Joint 4: Wrist 1 rotation (Y-axis)
        if (this.joints[3]) {
            this.joints[3].rotation.y = jointPositions[3];
        }
        
        // Joint 5: Wrist 2 rotation (Y-axis)
        if (this.joints[4]) {
            this.joints[4].rotation.y = jointPositions[4];
        }
        
        // Joint 6: Wrist 3 rotation (Y-axis)
        if (this.joints[5]) {
            this.joints[5].rotation.y = jointPositions[5];
        }
        
        // Update link rotations to follow joints
        this.updateLinkRotations(jointPositions);
    }
    
    updateLinkRotations(jointPositions) {
        // Update link rotations to follow joint movements
        // This creates a more realistic robot movement
        for (let i = 0; i < this.links.length; i++) {
            if (this.links[i] && jointPositions[i] !== undefined) {
                this.links[i].rotation.y = jointPositions[i];
            }
        }
    }
    
    onWindowResize() {
        this.camera.aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    }
    
    animate() {
        requestAnimationFrame(() => this.animate());
        
        if (this.renderer && this.scene && this.camera) {
            this.renderer.render(this.scene, this.camera);
        } else {
            console.warn('Cannot render: missing renderer, scene, or camera');
        }
    }
    
    // Public methods for external control
    resetView() {
        this.camera.position.set(2, 2, 2);
        this.camera.lookAt(0, 0, 0);
    }
    
    setCameraPosition(x, y, z) {
        this.camera.position.set(x, y, z);
        this.camera.lookAt(0, 0, 0);
    }
    
    getCurrentJointPositions() {
        return this.joints.map(joint => joint.rotation.y);
    }
    
    destroy() {
        // Cleanup method
        if (this.renderer) {
            this.renderer.dispose();
        }
        if (this.container && this.renderer) {
            this.container.removeChild(this.renderer.domElement);
        }
    }
}

// Export the class for ES6 modules
export { Robot3DScene };
