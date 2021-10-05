<template>
  <div class="control-container">
    <Navbar :title="'터틀봇 조작'" :left_icon="true" :right_text="'제어'"/>
    
    <div class="minimap" @click="isModalViewed=true">
      <SimulationMap :clickEnable="false" :size="'150'" v-if="!isModalViewed"/>
    </div>
    <ModalView @close-modal="isModalViewed=false" v-if="isModalViewed">
      <div class="my-modal-content">
        <span class="map-text">맵을 터치해 해당 위치로 이동하세요.</span>
        <SimulationMap :clickEnable="true" :size="'300'"/>
      </div>
    </ModalView>

    <ControlCam style="margin-bottom: 30px;"/>
    <div class="my-btn">물건 들기</div>

  <Controller style="margin-top: -230px;" />
  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import ModalView from '@/components/common/ModalView.vue'
import ControlCam from '@/components/Control/ControlCam.vue'
import SimulationMap from '@/components/common/SimulationMap.vue'
import Controller from '@/components/common/Controller.vue'

export default {
  name: 'Control',
  components: {
    Navbar,
    ControlCam,
    ModalView,
    SimulationMap,
    Controller,
  },
  data() {
    return {
      isModalViewed: false,
      isClickedMap: false,
    }
  },
}
</script>

<style lang="scss" scoped>
  @import "./Control.scss";
</style>