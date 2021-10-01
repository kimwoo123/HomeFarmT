<template>
  <div>
    <p class="m-0 path-text">감시경로</p>
    <div class="container">

      <div class="path-item">
        <div class="align">
          <font-awesome-icon :icon="['far', 'circle']" class="icon"/>
          <span>감시경로1</span>
          <button type ='button' @click="change_path(1)">이걸로 변경</button>
          <button type ='button' @click="btn_new_path_on(1)">이거 새로 만들기</button>
          <button type ='button' @click="btn_new_path_off">이거 끝내기</button>
        </div>
        <div class="align">
          <font-awesome-icon icon="edit" class="icon"/>
          <font-awesome-icon icon="trash-alt" class="icon"/>
        </div>
      </div>

      <div class="path-item">
        <div class="align">
          <font-awesome-icon icon="check-circle" class="icon"/>
          <span>감시경로2</span>
          <button type ='button' @click="change_path(2)">이걸로 변경</button>
          <button type ='button' @click="btn_new_path_on(2)">이거 새로 만들기</button>
          <button type ='button' @click="btn_new_path_off">이거 끝내기</button>
        </div>
        <div class="align">
          <font-awesome-icon icon="edit" class="icon"/>
          <font-awesome-icon icon="trash-alt" class="icon"/>
        </div>
      </div>
      
      <div class="path-item-empty">새로운 감시 경로 만들기</div>


    </div>
  </div>
</template>

<style lang="scss" scoped>
  @import './PatrolPathSetting.scss';
</style>

<script>
export default {
  name: 'PatrolPathSetting',
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
    }
  }
}
</script>