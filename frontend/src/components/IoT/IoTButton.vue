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
      console.log('켜기')
      item.status = true
    },
    turnOff(item) {
      console.log('끄기')
      item.status = false
    },
  }
}
</script>