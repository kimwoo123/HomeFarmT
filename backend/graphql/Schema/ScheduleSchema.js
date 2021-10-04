const { gql } = require('apollo-server');

const ScheduleSchema = gql`
  type Query {
    allSchedule: [Schedule]
    getSchedule: [Schedule]
  }

  type Mutation {
    createSchedule(schedule_time: String): Schedule
    updateSchedule(schedule_time: String): Schedule
    deleteSchedule(schedule_time: String): Schedule
  }

  type Schedule {
    scheduleid: Int
    schedule_time: String
  }

`;

module.exports = ScheduleSchema;
