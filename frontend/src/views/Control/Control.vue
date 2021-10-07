<template>
  <div class="control-container">
    <Navbar :title="'터틀봇 조작'" :left_icon="true" :right_text="'제어'" :left_push="'Home'" :right_push="'IoT'"/>
    
    <div class="minimap" @click="modalOpen()">
      <SimulationMap :clickEnable="false" :size="'150'" v-if="!isModalViewed"/>
    </div>
    <ModalView @close-modal="modalOff()" v-if="isModalViewed">
      <div class="my-modal-content">
        <span class="map-text">맵을 터치해 해당 위치로 이동하세요.</span>
        <SimulationMap :clickEnable="true" :size="'300'"/>
      </div>
    </ModalView>

    <ControlCam style="margin-bottom: 30px;"/>
    <div class="btn-group">
      <div class="my-small-btn" @click="btn_pick_up">물건 들기</div>
      <div class="my-small-btn" @click="btn_put_down">물건 놓기</div>
    </div>

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
  methods: {
    btn_pick_up() {
      let data = 1;
      this.$socket.emit('pickupToServer', data);
    },
    btn_put_down() {
      let data = 2;
      this.$socket.emit('putdownToServer', data);
    },
    modalOpen() {
      this.isModalViewed = true
      this.$socket.emit('ismapopenToServer', 1);
    },
    modalOff() {
      this.isModalViewed = false
      this.$socket.emit('ismapopenToServer', 0);
    },
  }
}
</script>

<style lang="scss" scoped>
  @import "./Control.scss";
</style>