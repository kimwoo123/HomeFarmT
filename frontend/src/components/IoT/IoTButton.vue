<template>
  <div class="iot-container">
    <div v-for="(item, idx) in items" :key="idx" class="iot-item" :class="{ 'iot-active' :item.status }"
      @click="item.status ? turnOff(item) : turnOn(item)">
      {{ item.name }}
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
        },
        {
          name: '에어컨2',
          status: false,
        },
        {
          name: '에어컨',
          status: false,
        },
        {
          name: '에어컨2',
          status: false,
        },
        {
          name: '에어컨',
          status: false,
        },
        {
          name: '에어컨2',
          status: false,
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