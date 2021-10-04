<template>
  <div>
    <Navbar :title="'감시'" :left_icon="true" :right_text="''"/>
    <PatrolCam style="margin-bottom: 25px;"/>
    <div class="patrol-container">
      <PatrolPath style="margin-bottom: 22px;"/>
      <PatrolHistory/>
    </div>

  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import PatrolCam from '@/components/Patrol/PatrolCam.vue'
import PatrolPath from '@/components/Patrol/PatrolPath.vue'
import PatrolHistory from '@/components/Patrol/PatrolHistory.vue'

export default {
  name: 'Patrol',
  components: {
    Navbar,
    PatrolCam,
    PatrolHistory,
    PatrolPath,
  }
}
</script>

<style lang="scss" scoped>
  @import './Patrol.scss';
</style>
