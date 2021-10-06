<template>
  <div class="container">
    <PatrolPathSetting/>
  </div>
</template>

<script>
import PatrolPathSetting from '@/components/Patrol/PatrolPathSetting.vue'

export default {
  name: 'PatrolSetting',
  components: {
    PatrolPathSetting,
  }
}
</script>

<style lang="scss" scoped>
  @import './PatrolSettinga.scss';
</style>