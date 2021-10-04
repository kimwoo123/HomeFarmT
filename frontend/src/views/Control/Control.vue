<template>
  <div>
    <Navbar :title="'터틀봇 조작'" :left_icon="true" :right_text="'제어'"/>
    
    <div class="minimap">
      <img src="@/assets/images/cam.png" alt="map" @click="isModalViewed=true" :height="100">
    </div>
    <ModalView @close-modal="isModalViewed=false" v-if="isModalViewed">
      <div class="my-modal-content">
        <span class="map-text">맵을 터치해 해당 위치로 이동하세요.</span>
      </div>
    </ModalView>

    <ControlCam style="margin-bottom: 30px;"/>
    <div class="my-btn">물건 들기</div>
  
  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import ModalView from '@/components/common/ModalView.vue'
import ControlCam from '@/components/Control/ControlCam.vue'

export default {
  name: 'Control',
  components: {
    Navbar,
    ControlCam,
    ModalView
  },
  data() {
    return {
      isModalViewed: false,
    }
  },
}
</script>

<style lang="scss" scoped>
  @import "./Control.scss";
</style>