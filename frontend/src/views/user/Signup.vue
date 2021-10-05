<template>
  <div class="bg">
    <Navbar :title="'회원가입'" :left_icon="true" :right_text="''" style="color: white;"/>
    <div class="signup-container">
      <div class="input-container"> 
        <div class="item">
          <span class="font-400">이메일</span>
          <input type="email" v-model="email" placeholder="이메일"> 
        </div>
        <div class="item">
          <span class="font-400">비밀번호</span>
          <input type="password" id="password" v-model="password" placeholder="비밀번호"> 
        </div>
        <div class="item">
          <span class="font-400">비밀번호 재입력</span>
          <input type="password" id="passwordConfirmation" v-model="passwordConfirmation" placeholder="비밀번호 재입력">
        </div>
      </div>

      <button class="user-btn" @click="requestSignup()">회원가입</button>
    </div>
  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import gql from 'graphql-tag'
// import { useMutation } from '@vue/apollo-composable'

export default {
  name: 'Signup',
  components: {
    Navbar,
  },
  data() {
    return {
      email: '',
      password: '',
      passwordConfirmation: '',
      error: {
				email: false,
				password: false,
				passwordConfirmation: false,
			},
    }
  },
  methods:{
    requestSignup() {
      this.$apollo.mutate({
        mutation: gql`mutation ($email: String!, $password: String) {
          signUp(email: $email , password: $password) {
            email
            password
          }
        }`, 
        variables: {
          email: this.email,
          password: this.password
        }
        })
        .then((res) => {
          console.log(res, 'done')
        })
        .catch((err) => {
          console.log(err, 'no')
        })
      },
    },
  }
</script>

<style lang="scss" scoped>
  @import './Signup.scss';
</style>
