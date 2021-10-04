<template>
  <div class="iot-container">
    <div v-for="(item, idx) in items" :key="idx" class="iot-item shadows" :class="{ 'iot-active' :item.status }"
      @click="item.status ? turnOff(item) : turnOn(item)">
      <img src="@/assets/icons/devices/light.svg" alt="light" class="icon">
      <span class="name">{{ item.name }}</span>
      <span class="position font-300" :class="{ 'position-active' :item.status }">{{ item.position }}</span>
    </div>
  </div>
</template>

<style lang="scss" scoped>
  @import './IoTButton.scss';
</style>

<script>
// [0] 신발장 조명
// 0 : 상태유지
// 1 : ON
// 2 : OFF
// [1] 방1 조명
// [2] 방2 조명
// [3] 방3 조명
// [4] 방4 조명
// [5] 주방 조명
// [6] 거실 조명
// [7] 방1 에어컨
// [8] 방2 에어컨
// [9] 방3 에어컨
// [10] 거실 에어컨
// [11] 공기 청정기
// [12] TV
// [13] 방1 커튼
// [14] 방2 커튼
// [15] 방3 커튼
// [16] 거실 커튼
export default {
  name: 'IoTButton',
  data() {
    return {
      items: [
        {
          name: '에어컨',
          status: false,
          position: '부엌'
        },
        {
          name: '에어컨2',
          status: false,
          position: '부엌'
        },
        {
          name: '에어컨',
          status: false,
          position: '부엌'
        },
        {
          name: '에어컨2',
          status: false,
          position: '거실'
        },
        {
          name: '에어컨',
          status: false,
          position: '거실'
        },
        {
          name: '에어컨2',
          status: false,
          position: '침실'
        },
      ]
    }
  },
  methods: {
    turnOn(item) {
      console.log('iot-control');
      let data = '0 1';
      this.$socket.emit('iot-control', data);
      item.status
    },
    turnOff(item) {
      console.log(item);
      let data = '0 1';
      this.$socket.emit('iot-control', data);
      item.status = false
    },
  }
}
</script>