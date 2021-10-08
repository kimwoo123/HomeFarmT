<template>
  <div class="bg">
    <Navbar :title="'로그인'" :left_icon="true" :right_text="''" style="color: white;"/>
    <div class="login-container">
      <div class="input-container">
        <div class="item">
          <span class="font-400">이메일</span>
          <input type="email" v-model="email" placeholder="이메일">
        </div>
        <div class="item">
          <span class="font-400">비밀번호</span>
          <input type="password" v-model="password" @keyup.enter="requestLogin" placeholder="비밀번호">
        </div>
        <button class="user-btn" @click="requestLogin()">Login</button>
      </div>
      <div @click="$router.push({ name: 'Signup' })" class="btn-signup">
        <span>처음 이용하시나요?</span>
        <font-awesome-icon icon="chevron-right"/>
      </div>
    </div>
  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import gql from 'graphql-tag'
import axios from 'axios'

export default {
  name: 'Login',
  components: {
    Navbar,
  },
  data() {
    return {
      email: '',
      password: '',
    }
  },
  methods: {
    done() {
      axios.get('http://localhost:3000/map1')
      .then(response => {
        console.log(response)
        console.log('done')
      })
      .catch(mapData => {
        console.log(mapData)
        console.log('not done')
      })
    },
    requestLogin() {
      this.$apollo.mutate({
        mutation: gql`mutation ($email: String!, $password: String!) {
          login(email: $email , password: $password) {
            region
            token
          }
        }`, 
        variables: {
          email: this.email,
          password: this.password
        }
        })
        .then((res) => {
          sessionStorage.setItem('token', res.data.login.token)
          sessionStorage.setItem('email', this.email)
          sessionStorage.setItem('region', res.data.login.region)
        })
        .then(() => {
          this.$router.push({ name: 'Home'})
        })
        .catch((err) => {
          alert(err.message.split(':')[1])
        })
      },
    },
  }
</script>

<style lang="scss" scoped>
  @import './Login.scss';
</style>