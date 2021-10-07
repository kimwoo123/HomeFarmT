<template>
  <div class="iot-container">
    <div v-for="(item, idx) in items" :key="idx" class="iot-item shadows" :class="{ 'iot-active' :item.status }"
      @click="item.status ? turnOff(item) : turnOn(item)">
      <img :src="`${ item.pic }`" class="icon" :id="`${ item.picName }`">
      <span class="name">{{ item.name }}</span>
      <span class="position font-300" :class="{ 'position-active' :item.status }">{{ item.position }}</span>
    </div>
  </div>
</template>
<style lang="scss" scoped>
  @import './IoTButton.scss';
</style>
<script>
export default {
  name: 'IoTButton',
  data() {
    return {
      items: [
        {
          name: '에어컨',
          status: false,
          grid: [220, 98],
          position: '거실',
          picName: 'aircon',
          pic: require('@/assets/icons/devices/aircon.svg')
        },
        {
          name: 'TV',
          status: false,
          grid: [155, 129],
          position: '거실',
          picName: 'tv',
          pic: require('@/assets/icons/devices/tv.svg')
        },
      ]
    }
  },
  methods: {
    turnOn(item) {
      console.log('iot-control');
      let data = '2' + ' ' + String(item.grid[0]) + ' ' + String(item.grid[1])
      this.$socket.emit('iot-control', data);
      item.status = true
      console.log(data)
      this.activeImg(item.picName)
    },
    turnOff(item) {
      console.log(item);
      let data = '2' + ' ' + String(item.grid[0]) + ' ' + String(item.grid[1])
      this.$socket.emit('iot-control', data);
      item.status = false
      this.originImg(item.picName)
    },
    activeImg(data) {
      let target = document.querySelector(`#${data}`)
      target.src = require(`@/assets/icons/devices/${data}_active.svg`)
    },
    originImg(data) {
      let target = document.querySelector(`#${data}`)
      target.src = require(`@/assets/icons/devices/${data}.svg`)
    },
  }
}
</script>

