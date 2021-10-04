<template>
  <div>
    <form>
      <!-- 로직 1. textarea 생성 -->
      <p>Security Status</p>
      <textarea id = "tSafetyStatus" cols = 30 rows = 2></textarea>

      <br>

      <!-- 로직 2. 침입자 인지 시점 이미지 뷰어 생성 -->
      <!-- <img src="cam.jpg?t=" width='480' onload='setTimeout(function() {src = src.substring(0, (src.lastIndexOf("t=")+2))+(new Date()).getTime()}, 50)'
      onerror='setTimeout(function() {src = src.substring(0, (src.lastIndexOf("t=")+2))+(new Date()).getTime()}, 50)'/> -->

      
      <!-- 로직 3. 순찰 모드 스위치 버튼 생성 -->
      <p>Patrol On/Off Status</p>
      <textarea id = "tPatrolStatus" cols = 30 rows = 2></textarea>
      <br>
      <button type ='button' id="A1-btn" @click="btn_patrol_on">On</button>
      <button type ='button' id="A2-btn" @click="btn_patrol_off">Off</button>
      
      <br>
      <!-- 로직 4. 수동 컨트롤 버튼 생성 -->
      <p>Manual Controller</p>
      <button type ='button' id="A1-btn" @mousedown="btn_turn_left" @mouseup="stop_interval">Turn Left</button>
      <button type ='button' id="A2-btn" @mousedown="btn_go_straight" @mouseup="stop_interval">Go Straight</button>
      <button type ='button' id="A2-btn" @mousedown="btn_turn_right" @mouseup="stop_interval">Turn Right</button>

      <button type ='button' @click="btn_new_path_on">시작</button>
      <button type ='button' @click="btn_new_path_off">끝</button>

    </form>
  </div>
</template>

<script>
export default {
  name: 'Test',
  data() {
    return {
      inter: '',
      isModalViewed: false,
    }
  },
  created() {

    this.$socket.on('disconnect', function()  {
        console.log('disconnected form server_client.');
    });


    // 로직 1. 서버에서 온 메시지를 웹페이지에 전달
    this.$socket.on('sendSafetyStatus', function(message) {
        console.log('sendSafetyStatus', message);
        document.querySelector('#tSafetyStatus').value = message;
    });

    this.$socket.on('sendPatrolStatus', function(message) {
        console.log('sendPatrolStatus', message);
        document.querySelector('#tPatrolStatus').value = message;
    });
  },
  methods: {
    btn_patrol_on() {
      console.log('iot-control');
      let data = '0 1';
      this.$socket.emit('iot-control', data);
    },
    btn_patrol_off() {
      console.log('btn_patrol_off');
      let data = 0;
      this.$socket.emit('PatrolOffToServer', data);
    },
    btn_turn_left() {
      console.log('btn_left');
      let data = 1;
      this.inter = setInterval(() => {
        this.$socket.emit('turnleftToServer', data);
      }, 100)
    },
    btn_go_straight() {
      console.log('btn_go_straight');
      let data = 2;
      this.inter = setInterval(() => {
        this.$socket.emit('gostraightToServer', data);
      }, 100)
    },
    btn_turn_right() {
      console.log('btn_turn_right');
      let data = 3;
      this.inter = setInterval(() => {
        this.$socket.emit('turnrightToServer', data);
      }, 100)
    },
    stop_interval() {
      clearInterval(this.inter)
    },
    btn_new_path_on() {
      console.log('new_path_on');
      let data = 3;
      this.$socket.emit('newPathOnToServer', data);
    },
    btn_new_path_off() {
      console.log('new_path_off');
      let data = 3;
      this.$socket.emit('newPathOffToServer', data);
    },
    modalOn() {
      this.isModalViewed = true
    }, 
    modalOff() {
      this.isModalViewed = false
    }
  }
}
</script>

<style>

</style>