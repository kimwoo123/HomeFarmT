const express = require('express');
const router = express.Router();
const schedule = require('node-schedule')
const fs = require('fs');
const { spawn } = require('child_process')


/* GET home page. */
router.get('/map1', function(req, res) {
  fs.readFile("./public/maps/map1.txt", "utf8", function(err, html) {
    res.send(html);
  })
});

// router.get('/patrol', getTest);
// function getTest(req, res) {
//   let rule = new schedule.RecurrenceRule()
//   let job = schedule.scheduleJob('10 * * * * *', function () {
//     console.log('schedule work')
//   })
// }

// 	socket.on('cleanerOnToServer', () => {
//         const opt = {
//             shell: true,
//             cwd: 'C:/Users/multicampus/Desktop/S05P21B202/ros2_smart_home/src/sub2/sub2/'
//         }
//         const child = spawn('call C:/dev/ros2_eloquent/setup.bat && call C:/Users/multicampus/Desktop/S05P21B202/ros2_smart_home/install/local_setup.bat && load_map.py', opt)
//         child.stderr.on('data', function (data) {
//             console.error("STDERR:", data.toString());
//           });
//           child.stdout.on('data', function (data) {
//             console.log("STDOUT:", data.toString());
//           });
//           child.on('exit', function (exitCode) {
//             console.log("Child exited with code: " + exitCode);
//           });
        
//         // socket.to(roomName).emit('cleanerOn'); // 일단 소켓에 cleanerOn을 보내긴 하는데 안쓸 수도?
//     })


// console.log(this.req, 'here')
module.exports = router;
