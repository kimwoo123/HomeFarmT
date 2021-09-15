<template>
  <div class="patrol-container">
    <PatrolCam/>
    <PatrolMenu/>
  </div>
</template>
<style>
  @import './Patrol.scss';
</style>
<script>
import PatrolCam from '../components/Patrol/PatrolCam.vue'
import PatrolMenu from '../components/Patrol/PatrolMenu.vue'

export default {
  name: 'Patrol',
  components: {
    PatrolCam,
    PatrolMenu,
  }

}
</script>

