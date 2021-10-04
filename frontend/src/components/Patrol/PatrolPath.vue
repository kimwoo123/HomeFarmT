<template>
  <div class="patrol-path shadows">
    <div class="text-content">
      <div>
        <span class="target-path">{{ path }}</span>
        <span>로 감시를 실행합니다.</span>
      </div>
      <span v-if="!status" class="status font-300">상태 이상없음</span>
      <span v-else class="status font-300">상태 이상 발생</span>
    </div>
    <img src="@/assets/icons/switch.svg" alt="switch">
  </div>
</template>

<script>
export default {
  name: 'PatrolPath',
  data() {
    return {
      path: '1번 경로',
      status: false,
    }
  }
}
</script>

<style lang="scss" scoped>
  @import './PatrolPath.scss';
</style>