<template>
  <div class="controller-container">
    <font-awesome-icon icon="chevron-up" class="item shadows" @mousedown="btn_go_straight" @mouseup="stop_interval"/>
    <div class="group">
      <font-awesome-icon icon="chevron-left" class="item shadows" @mousedown="btn_turn_left" @mouseup="stop_interval"/>
      <font-awesome-icon icon="chevron-down" class="item shadows"/>
      <font-awesome-icon icon="chevron-right" class="item shadows"  @mousedown="btn_turn_right" @mouseup="stop_interval"/>
    </div>
  </div>
</template>

<script>
export default {
  name: 'Controller',
  data() {
    return {
      inter: ''
    }
  },
  created() {
    this.$socket.on('disconnect', function()  {
        console.log('disconnected form server_client.');
    });
    this.$socket.emit('PatrolOffToServer', 0);
  },
  methods: {
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
  }
}
</script>

<style lang="scss" scoped>
  @import './Controller.scss';
</style>