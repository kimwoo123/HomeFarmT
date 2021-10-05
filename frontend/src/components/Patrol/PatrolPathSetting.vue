<template>
  <div>
    감시 
    
    <textarea id = "tPatrolStatus" cols = 30 rows = 2></textarea>
    <button type ='button' @click="btn_patrol_on">On</button>
    <button type ='button' @click="btn_patrol_off">Off</button>
    <hr>
    <p class="m-0 path-text">감시경로</p>
    <div class="container">

      <div class="path-item">
        <div class="align">
          <span>감시경로1</span>
          <button type ='button' @click="change_path(1)">이걸로 변경</button>
          <button type ='button' @click="btn_new_path_on(1)">이거 새로 만들기</button>
          <button type ='button' @click="btn_new_path_off">이거 끝내기</button>
        </div>
      </div>

      <div class="path-item">
        <div class="align">
          <span>감시경로2</span>
          <button type ='button' @click="change_path(2)">이걸로 변경</button>
          <button type ='button' @click="btn_new_path_on(2)">이거 새로 만들기</button>
          <button type ='button' @click="btn_new_path_off">이거 끝내기</button>
        </div>
      </div>
      
      <div class="path-item">
        <div class="align">
          <span>감시경로3</span>
          <button type ='button' @click="change_path(3)">이걸로 변경</button>
          <button type ='button' @click="btn_new_path_on(3)">이거 새로 만들기</button>
          <button type ='button' @click="btn_new_path_off">이거 끝내기</button>
        </div>
      </div>


    </div>
  </div>
</template>

<style lang="scss" scoped>
  @import './PatrolPathSetting.scss';
</style>

<script>
export default {
  name: 'PatrolPathSetting',
  created() {
    this.$socket.on('sendPatrolStatus', function(message) {
        console.log('sendPatrolStatus', message);
        document.querySelector('#tPatrolStatus').value = message;
    });
  },
  methods: {
    change_path(data) {
      this.$socket.emit('changePathToServer', data);
    },
    btn_new_path_on(data) {
      console.log('new_path_on');
      this.$socket.emit('newPathOnToServer', data);
    },
    btn_new_path_off() {
      console.log('new_path_off');
      this.$socket.emit('newPathOffToServer');
    },
    btn_patrol_on() {
      console.log('btn_patrol_on');
      let data = 1;
      this.$socket.emit('PatrolOnToServer', data);
    },
    btn_patrol_off() {
      console.log('btn_patrol_off');
      let data = 0;
      this.$socket.emit('PatrolOffToServer', data);
    },
  }
}
</script>