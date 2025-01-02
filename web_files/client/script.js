window.onload = function() {

    // TODO: Move useer Parameters to JSON/YAML file
    // TODO: Add handedness and dual hand control, run off right hand for now
    // TODO: Add anglular position and velocity logging
    // TODO: Add more failsafes for whether different components are active


    // =============== //
    // User Parameters //
    // =============== //

    base_position_robot = {x: 0.2, y: 0.0, z: 1.0};


    // =============== //
    // State Variables //
    // =============== //

    var endeff_pos = null;
    var tracking_active = false;
    var leftPosition_robot = null;
    var rightPosition_robot = null;
    var leftVelocity_robot = null;
    var rightVelocity_robot = null;


    // ============== //
    // AFRAME OBJECTS //
    // ============== //

    // Controllers
    const leftController = document.getElementById("left_hand");
    const rightController = document.getElementById("right_hand");
    // Base frame
    const base_origin = document.getElementById("base_origin");
    const base_x_axis = document.getElementById("base_x");
    const base_y_axis = document.getElementById("base_y");
    const base_z_axis = document.getElementById("base_z");
    // End effector
    const endeff_pos_marker = document.getElementById("endeff_pos");

    // =============== //
    // ROSBRIDGE SETUP //
    // =============== //

    // Create ros object to communicate over your Rosbridge connection
    const ros = new ROSLIB.Ros({ url: "wss://192.168.1.164:9090" });

    // When the Rosbridge server connects, fill the span with id "status" with "successful"
    ros.on("connection", () => {
        console.log("ROS Connected");
    });

    // When the Rosbridge server experiences an error, fill the "status" span with the returned error
    ros.on("error", (error) => {
        console.log("ROS Error");
    });

    // When the Rosbridge server shuts down, fill the "status" span with "closed"
    ros.on("close", () => {
        console.log("ROS Closed");
    });


    // ==================================== //
    // ROBOT BASE AND ENDEFF REPRESENTATION //
    // ==================================== //

    // Robot base position
    var base_position_aframe = robot_coords_to_aframe(base_position_robot);
    base_origin.setAttribute('position', base_position_aframe);
    base_x_axis.setAttribute('position', addVec(base_position_aframe, {x: 0.15, y: 0, z: 0}));
    base_y_axis.setAttribute('position', addVec(base_position_aframe, {x: 0, y: 0, z: -0.15}));
    base_z_axis.setAttribute('position', addVec(base_position_aframe, {x: 0, y: 0.15, z: 0}));

    // End effector position subscriber
    const endeff_pos_sub = new ROSLIB.Topic({ ros: ros, name: "/endeff_pos", messageType: "geometry_msgs/Point" });
    endeff_pos_sub.subscribe(function(message) {
        // Relocate sphere to recieved coordinates
        endeff_pos_marker.setAttribute('position', addVec(base_position_aframe, robot_coords_to_aframe(message)));
        endeff_pos = message
    });

    // Tracking Mode
    rightController.addEventListener("abuttonup", function (event) {
        if(endeff_pos !== null && relNorm(endeff_pos, rightPosition_robot) < 0.03) { //&& absNorm(rightVelocity_robot) <= 0.02) {
            tracking_active = true;
            endeff_pos_marker.setAttribute('color', 'green');
        }
    });
    rightController.addEventListener("bbuttonup", function (event) {
        tracking_active = false;
        endeff_pos_marker.setAttribute('color', 'red');
    });


    // ================== //
    // Controller Logging //
    // ================== //

    // Controller position and velocity publishers
    const left_pos_pub  = new ROSLIB.Topic({ ros: ros, name: "/left_controller_pos", messageType: "geometry_msgs/Point" });
    const right_pos_pub = new ROSLIB.Topic({ ros: ros, name: "/right_controller_pos", messageType: "geometry_msgs/Point" });
    const left_vel_pub  = new ROSLIB.Topic({ ros: ros, name: "/left_controller_vel", messageType: "geometry_msgs/Point" });
    const right_vel_pub = new ROSLIB.Topic({ ros: ros, name: "/right_controller_vel", messageType: "geometry_msgs/Point" });
    const tracking_active_pub = new ROSLIB.Topic({ ros: ros, name: "/tracking_active", messageType: "std_msgs/Bool" });

    // Controller Position Logger with Velocity, Low-Pass Filter, and Coordinate Conversion
    AFRAME.registerComponent('log-controller-position', {
        schema: {
            smoothingFactor: { type: 'number', default: 0.5 } // Adjust the smoothing factor
        },
        init: function () {
            this.leftPrevPos = new THREE.Vector3();
            this.rightPrevPos = new THREE.Vector3();
            this.leftFilteredVelocity = new THREE.Vector3(); // Store the filtered velocity
            this.rightFilteredVelocity = new THREE.Vector3(); // Store the filtered velocity
            this.lastTime = performance.now();
        },
        tick: function () {
            const currentTime = performance.now();
            const deltaTime = (currentTime - this.lastTime) / 1000; // Time in seconds

            // Log left controller position and velocity
            if (leftController) {
                const leftPosition_aframe = leftController.getAttribute('position');
                const leftPosition = new THREE.Vector3(leftPosition_aframe.x - base_position_aframe.x,
                    leftPosition_aframe.y - base_position_aframe.y, 
                    leftPosition_aframe.z - base_position_aframe.z);
                
                // Calculate velocity
                const leftVelocity = leftPosition.clone().sub(this.leftPrevPos).divideScalar(deltaTime);
                
                // Apply low-pass filter to velocity
                this.leftFilteredVelocity.lerp(leftVelocity, this.data.smoothingFactor);
                
                // Convert position and filtered velocity to robot coordinates
                leftPosition_robot = aframe_coords_to_robot(leftPosition);
                leftVelocity_robot = aframe_coords_to_robot(this.leftFilteredVelocity);

                // Publish position and filtered velocity (in robot coordinates)
                left_pos_pub.publish(leftPosition_robot);
                left_vel_pub.publish(leftVelocity_robot);

                // Update previous position
                this.leftPrevPos.copy(leftPosition);
            }

            // Log right controller position and velocity
            if (rightController) {
                const rightPosition_aframe = rightController.getAttribute('position');
                const rightPosition = new THREE.Vector3(rightPosition_aframe.x - base_position_aframe.x, 
                    rightPosition_aframe.y - base_position_aframe.y, 
                    rightPosition_aframe.z - base_position_aframe.z);
                
                // Calculate velocity
                const rightVelocity = rightPosition.clone().sub(this.rightPrevPos).divideScalar(deltaTime);
                
                // Apply low-pass filter to velocity
                this.rightFilteredVelocity.lerp(rightVelocity, this.data.smoothingFactor);
                
                // Convert position and filtered velocity to robot coordinates
                rightPosition_robot = aframe_coords_to_robot(rightPosition);
                rightVelocity_robot = aframe_coords_to_robot(this.rightFilteredVelocity);

                // Publish position and filtered velocity (in robot coordinates)
                right_pos_pub.publish(rightPosition_robot);
                right_vel_pub.publish(rightVelocity_robot);

                // Update previous position
                this.rightPrevPos.copy(rightPosition);
            }

            // Report whether tracking is active
            tracking_active_msg = new ROSLIB.Message({data: tracking_active});
            tracking_active_pub.publish(tracking_active_msg);

            // Update the last time for the next frame
            this.lastTime = currentTime;
        }
    });

    

    // Attach the component to the scene to track controller positions
    document.querySelector('a-scene').setAttribute('log-controller-position', '');

}


// ============================================== //
// Coordinate and Vector Related Helper Functions //
// ============================================== //

function aframe_coords_to_robot(aframe_coords) {
    robot_coords = new ROSLIB.Message({
        x: aframe_coords.x,
        y: -aframe_coords.z,
        z: aframe_coords.y
    });
    return robot_coords
}

function robot_coords_to_aframe(robot_coords) {
    aframe_coords = {
        x: robot_coords.x,
        y: robot_coords.z,
        z: -robot_coords.y
    }
    return aframe_coords
}

function addVec(vecA, vecB) {
    return {
        x: vecA.x + vecB.x,
        y: vecA.y + vecB.y,
        z: vecA.z + vecB.z,
    }
}

function subVec(vecA, vecB) {
    return {
        x: vecA.x - vecB.x,
        y: vecA.y - vecB.y,
        z: vecA.z - vecB.z,
    }
}

function relNorm(vecA, vecB) {
    const dx = vecA.x - vecB.x;
    const dy = vecA.y - vecB.y;
    const dz = vecA.z - vecB.z;
    
    // Calculate the Euclidean distance
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

function absNorm(vec) {
    const x = vec.x;
    const y = vec.y;
    const z = vec.z;
    
    // Calculate the norm (magnitude) of the vector
    return Math.sqrt(x * x + y * y + z * z);
}