<template>
  <div>

    <div v-for="(item, idx) in items" :key="idx" 
    class="iot-control-box" :class="{ 'iot-active' :item.status }"
    @click="item.status ? turnOff(item) : turnOn(item)">
      {{ item.name }}
    </div>
  
  </div>
</template>

<style lang="scss" scoped>
  @import url('./IoT.scss');
</style>

<script>
export default {
  name: 'IoT',
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
      ]
    }
  },
  methods: {
    turnOn(item) {
      console.log('켜기')
      console.log(item)
      item.status = true
    },
    turnOff(item) {
      console.log('끄기')
      console.log(item)
      item.status = false
    },
  }
}
</script>