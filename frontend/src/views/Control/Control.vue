<template>
  <div>
    <Navbar :title="'터틀봇 조작'" :left_icon="true" :right_text="'제어'"/>
    <ControlCam style="margin-bottom: 30px;"/>
    <div class="my-btn">물건 들기</div>
  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import ControlCam from '@/components/Control/ControlCam.vue'

export default {
  name: 'Control',
  components: {
    Navbar,
    ControlCam
  }
}
</script>

<style lang="scss" scoped>
  @import "./Control.scss";
</style>