<template>
  <div class="patrol-menu-container">

    <div>
      감시하기
      <div v-if="patrol" @click="stopPatrol" class="patrol-menu-box box-filled">
        감시하는 중
      </div>
      <div v-else @click="startPatrol" class="patrol-menu-box not-patrol">
        감시 시작하기
      </div>
    </div>

    <div>
      이상 감지 녹화
      <div class="patrol-menu-box abnormal-video-list">
        <figure
          class="abnormal-video-figure"
          v-for="url, idx in abnormalList"
          :key="idx">
          <img class="abnormal-video-img" :src="url">
        </figure>
      </div>
    </div>

  </div>
</template>
<style lang="scss" scoped>
  @import './PatrolMenu.scss';
</style>
<script>
export default {
  name: 'PatrolMenu',
  data() {
    return {
      abnormalList: [
        'http://localhost:8080/img/cam.a03eeead.png',
        'http://localhost:8080/img/cam.a03eeead.png',
      ],
      patrol: true,
    }
  },
  methods: {
    stopPatrol() {
      this.patrol = false
    },
    startPatrol() {
      this.patrol = true
    }
  }
}
</script>

