<template>
  <div class="modal modal-overlay" @click.self="$emit('close-modal')">
    <div class="my-modal-window">
      <div class="icon">
        <font-awesome-icon icon="times" @click="$emit('close-modal')" />
      </div>
      <slot/>
    </div>
  </div>
</template>

<style lang="scss" scoped>
.modal {
  &.modal-overlay {
    display: flex;
    align-items: center;
    justify-content: center;
    position: fixed;
    z-index: 30;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
    background: rgba(0, 0, 0, 0.5);
  }

  .my-modal-window {
    background: #fff;
    overflow: hidden;
    border-radius: 10px;
    padding: 16px;
  }
}

.icon {
  font-size: 18px;
  display: flex;
  justify-content: flex-end;
  color: $gray;
  cursor: pointer
}
</style>