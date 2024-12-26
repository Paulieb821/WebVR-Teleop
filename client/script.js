window.onload = function() {

    // Setup
    var socket = io();

    // Headset connection
    socket.emit('headset_connect');

    // Controller Position Logger with Velocity, Low-Pass Filter, and Coordinate Conversion
    AFRAME.registerComponent('log-controller-position', {
        schema: {
            leftControllerId: { type: 'string', default: '#left_hand' },
            rightControllerId: { type: 'string', default: '#right_hand' },
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
            const leftController = document.querySelector(this.data.leftControllerId);
            const rightController = document.querySelector(this.data.rightControllerId);
            const currentTime = performance.now();
            const deltaTime = (currentTime - this.lastTime) / 1000; // Time in seconds

            // Log left controller position and velocity
            if (leftController) {
                const leftPosition_aframe = leftController.getAttribute('position');
                const leftPosition = new THREE.Vector3(leftPosition_aframe.x, leftPosition_aframe.y, leftPosition_aframe.z);
                
                // Calculate velocity
                const leftVelocity = leftPosition.clone().sub(this.leftPrevPos).divideScalar(deltaTime);
                
                // Apply low-pass filter to velocity
                this.leftFilteredVelocity.lerp(leftVelocity, this.data.smoothingFactor);
                
                // Convert position and filtered velocity to robot coordinates
                const leftPosition_robot = aframe_coords_to_robot(leftPosition_aframe);
                const leftVelocity_robot = aframe_coords_to_robot(this.leftFilteredVelocity);

                // Emit position and filtered velocity (in robot coordinates)
                socket.emit('leftPos', leftPosition_robot);
                socket.emit('leftVel', leftVelocity_robot); // Send the filtered and converted velocity

                // Update previous position
                this.leftPrevPos.copy(leftPosition);
            }

            // Log right controller position and velocity
            if (rightController) {
                const rightPosition_aframe = rightController.getAttribute('position');
                const rightPosition = new THREE.Vector3(rightPosition_aframe.x, rightPosition_aframe.y, rightPosition_aframe.z);
                
                // Calculate velocity
                const rightVelocity = rightPosition.clone().sub(this.rightPrevPos).divideScalar(deltaTime);
                
                // Apply low-pass filter to velocity
                this.rightFilteredVelocity.lerp(rightVelocity, this.data.smoothingFactor);
                
                // Convert position and filtered velocity to robot coordinates
                const rightPosition_robot = aframe_coords_to_robot(rightPosition_aframe);
                const rightVelocity_robot = aframe_coords_to_robot(this.rightFilteredVelocity);

                // Emit position and filtered velocity (in robot coordinates)
                socket.emit('rightPos', rightPosition_robot);
                socket.emit('rightVel', rightVelocity_robot); // Send the filtered and converted velocity

                // Update previous position
                this.rightPrevPos.copy(rightPosition);
            }

            // Update the last time for the next frame
            this.lastTime = currentTime;
        }
    });

    // Attach the component to the scene to track controller positions
    document.querySelector('a-scene').setAttribute('log-controller-position', '');

    // Emit a message when the page is closed or refreshed
    window.onbeforeunload = function() {
        // Emit "disconnect" or any other event you want when the page is closed
        socket.emit('headset_disconnect');
    };

}

// Coordinate conversion system
function aframe_coords_to_robot(aframe_coords) {
    robot_coords = {
        x: aframe_coords.x,
        y: -aframe_coords.z,
        z: aframe_coords.y
    }
    return robot_coords
}

function robot_coords_to_aframe(robot_coords) {
    aframe_coords = {
        x: robot_coords.x,
        y: robot_coords.z,
        z: -robot_coords.coords.y
    }
    return aframe_coords
}
