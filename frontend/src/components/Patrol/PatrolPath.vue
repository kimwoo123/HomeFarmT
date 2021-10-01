<template>
  <div>
    <p class="m-0 path-text">감시경로</p>
    <div class="container">
      <div class="path-item">
        <div class="align">
          <font-awesome-icon :icon="['far', 'circle']" class="icon"/>
          <span>감시경로1</span>
        </div>
        <div class="align">
          <font-awesome-icon icon="edit" class="icon"/>
          <font-awesome-icon icon="trash-alt" class="icon"/>
        </div>
      </div>
      <div class="path-item">
        <div class="align">
          <font-awesome-icon icon="check-circle" class="icon"/>
          <span>감시경로2</span>
        </div>
        <div class="align">
          <font-awesome-icon icon="edit" class="icon"/>
          <font-awesome-icon icon="trash-alt" class="icon"/>
        </div>
      </div>
      <div class="path-item-empty">새로운 감시 경로 만들기</div>
    </div>
  </div>
</template>

<style lang="scss" scoped>
  @import './PatrolPath.scss';
</style>

<script>
export default {
  name: 'PatrolPath'
}
</script>