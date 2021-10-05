<template>
  <div>
    
    <div v-if="!deviceAround">
      <Navbar :title="'기기 추가'" :left_icon="true" :right_text="''" :left_push="'IoT'" :right_push="''"/>
      <div>
        <div class="move_device">
          <img src="@/assets/icons/new_device.svg" alt="move_device">
          <span>기기 주위로 이동하세요.</span>
        </div>
        <div class="chevron-btn" @click="deviceAround = true">
          <div class="box"></div>
          <span>다음</span>
          <font-awesome-icon icon="chevron-right"/>
        </div>
      </div>
    </div>


    <div v-else>
      <Navbar :title="'기기 추가'" :left_icon="true" :right_text="'완료'"/>
      <div>
        <div class="find-device">
          <span>기기를 찾았습니다.</span>
          <span>버튼을 눌러 연결을 완료하세요.</span>
        </div>
        <div class="IoTNewcard-container">
          <IoTNewCard v-for="(device, idx) in devices" :key="idx" :device="device"/>
        </div>
      </div>
    </div>

  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import IoTNewCard from '@/components/IoT/IoTNewCard.vue'

export default {
  name: 'IoTNew',
  components: {
    Navbar,
    IoTNewCard,
  },
  data() {
    return {
      deviceAround: false,
      devices: [
        {
          name: '형광등',
          position: '부엌'
        },
        {
          name: '형광등',
          position: '부엌'
        },
        {
          name: '형광등',
          position: '부엌'
        },
        {
          name: '형광등',
          position: '부엌'
        },
        {
          name: '형광등',
          position: '부엌'
        },
        {
          name: '형광등',
          position: '부엌'
        },
      ]
    }
  },
}
</script>

<style lang="scss" scoped>
  @import "./IoTNew.scss";
</style>