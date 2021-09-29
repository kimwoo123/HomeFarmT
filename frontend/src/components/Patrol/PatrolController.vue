<template>
  <div>
    <font-awesome-icon icon="arrow-left" class="icon" @mousedown="btn_turn_left" @mouseup="stop_interval"/>
    <font-awesome-icon icon="arrow-up" class="up icon" @mousedown="btn_go_straight" @mouseup="stop_interval"/>
    <font-awesome-icon icon="arrow-right" class="icon" @mousedown="btn_turn_right" @mouseup="stop_interval"/>
  </div>
</template>

<style lang="scss" scoped>
  @import './PatrolController.scss';
</style>

<script>
export default {
  name: 'PatrolController',
  props: {
    data: String
  },
  data() {
    return {
      da: '',
      inter: ''
    }
  },
  created() {
    this.da = this.data
    console.log(this.da)
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